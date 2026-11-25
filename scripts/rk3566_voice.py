"""Audio input and model helpers for the RK3566 speech tests."""

import argparse
from contextlib import contextmanager
import ctypes
from itertools import chain
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
from tempfile import NamedTemporaryFile
from threading import Thread
import time
import wave

SCRIPT_DIR = Path(__file__).resolve().parent
# Source launchers and the installed test programs share the deployed runtime.
VOICE_DIR = SCRIPT_DIR
if SCRIPT_DIR.name == "scripts" and (SCRIPT_DIR.parent / "target/voice-demo/runtime").is_dir():
    VOICE_DIR = SCRIPT_DIR.parent / "target/voice-demo"
for dependency in (VOICE_DIR / "runtime/python", VOICE_DIR / "tts/frontend"):
    if dependency.is_dir():
        sys.path.insert(0, str(dependency))

import numpy as np


def speech_runtime(root):
    runtime = root / "runtime"
    return runtime if runtime.is_dir() else VOICE_DIR / "runtime"


def load_sherpa(root, *, use_rknn):
    runtime = speech_runtime(root)
    module = runtime / "python"
    if use_rknn:
        module = runtime / "python-rknn"
        library = runtime / "lib/librknnrt.so"
        if not module.is_dir() or not library.is_file():
            raise RuntimeError(f"RKNN Sherpa-ONNX runtime is missing from {runtime}")
        ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
    if module.is_dir() and str(module) not in sys.path:
        sys.path.insert(0, str(module))
    try:
        import sherpa_onnx
    except ModuleNotFoundError as error:
        if error.name != "sherpa_onnx":
            raise
        raise RuntimeError(f"Sherpa-ONNX is missing from {speech_runtime(root)}") from error
    return sherpa_onnx


SAMPLE_RATE = 16000
VAD_WINDOW = 512


def test_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", type=Path, default=VOICE_DIR / "rk3566-voice-demo.json")
    return parser


def add_microphone_arguments(parser, *, source=None):
    parser.add_argument("--source", default=source, help="Override the configured PulseAudio microphone")
    parser.epilog = "Press Ctrl+C to stop listening."


def load_config(path, *, section=None, overrides=None):
    path = path.resolve()
    config = json.loads(path.read_text(encoding="utf-8"))
    if section is not None:
        model = config.get(section, {})
        if section == "vad" and isinstance(model, str):
            model = {"model": model}
        model.update({key: value for key, value in (overrides or {}).items() if value is not None})
        config[section] = model

    active = config if section is None else {section: config[section]}
    use_rknn = active.get("tts", {}).get("type") == "vits-rknn"
    for name in ("kws", "vad", "asr"):
        model = active.get(name, {})
        if isinstance(model, dict) and model.get("provider") == "rknn":
            use_rknn = True
            break

    global sherpa
    sherpa = load_sherpa(path.parent, use_rknn=use_rknn)
    return path.parent, config


def available_memory_mib():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return round(int(line.split()[1]) / 1024, 1)


def emit(event, **fields):
    print(json.dumps({"event": event, **fields}, ensure_ascii=False), flush=True)


def peak_rss_mib():
    return round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1)


def elapsed_ms(started):
    return round((time.perf_counter() - started) * 1000, 1)


def model_files(root, model, *keys):
    return {key: str(root / model[key]) for key in keys}


def microphone_frames(capture):
    while True:
        frame = capture.stdout.read(2 * VAD_WINDOW)
        if len(frame) != 2 * VAD_WINDOW:
            raise RuntimeError(f"parec stopped during capture (exit code {capture.wait()})")
        yield np.frombuffer(frame, dtype="<i2").astype(np.float32) / 32768


@contextmanager
def open_microphone(args, config):
    source = args.source or config.get("microphone", "@DEFAULT_SOURCE@")
    command = [
        "parec", "--raw", "--format=s16le", "--channels=1", "--rate=16000",
        "--latency-msec=40", "--client-name=WonderDuck voice test", "--device=" + source,
    ]
    # The test program handles Ctrl+C and closes the capture process in finally.
    with subprocess.Popen(command, stdout=subprocess.PIPE, start_new_session=True) as capture:
        try:
            frames = microphone_frames(capture)
            yield source, chain((next(frames),), frames)
        finally:
            if capture.poll() is None:
                capture.terminate()
                try:
                    capture.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    capture.kill()
                    capture.wait()


def speech_threads(config, provider):
    # Sherpa passes this value to RKNN as a core selection. Use automatic
    # selection for the RK3566's single core.
    return 1 if provider == "rknn" else config.get("threads", 1)


def load_spotter(root, config, *, keywords=None):
    provider = config.get("provider", "cpu")
    settings = dict(
        **model_files(root, config, "tokens", "encoder", "decoder", "joiner"),
        provider=provider,
        num_threads=speech_threads(config, provider),
        keywords_threshold=config.get("threshold", 0.25),
        keywords_score=config.get("score", 1.0),
        num_trailing_blanks=config.get("trailing_blanks", 1),
    )
    emit("kws_backend", provider=provider)
    if keywords is None:
        return sherpa.KeywordSpotter(**settings, keywords_file=str(root / config["keywords_file"]))

    if not keywords.strip():
        raise ValueError("KEYWORDS must contain a Sherpa-ONNX keyword entry")
    # Sherpa reads the keyword file when the spotter is constructed.
    with NamedTemporaryFile(mode="w", encoding="utf-8") as keyword_file:
        keyword_file.write(keywords.strip() + "\n")
        keyword_file.flush()
        return sherpa.KeywordSpotter(**settings, keywords_file=keyword_file.name)


def detect_keyword(spotter, stream):
    while spotter.is_ready(stream):
        spotter.decode_stream(stream)
        keyword = spotter.get_result(stream)
        if keyword:
            return keyword
    return ""


def finish_spotting(spotter, stream):
    stream.accept_waveform(SAMPLE_RATE, np.zeros(SAMPLE_RATE, dtype=np.float32))
    stream.input_finished()
    return detect_keyword(spotter, stream)


def emit_speech(segment, offset=0):
    samples = segment.samples
    start = offset + segment.start
    emit("speech", start_s=round(start / SAMPLE_RATE, 3),
         end_s=round((start + len(samples)) / SAMPLE_RATE, 3),
         audio_s=round(len(samples) / SAMPLE_RATE, 3))
    return samples


def make_vad(root, model):
    if isinstance(model, str):
        model = {"model": model}
    config = sherpa.VadModelConfig()
    config.sample_rate = SAMPLE_RATE
    config.provider = model.get("provider", "cpu")
    config.num_threads = speech_threads(model, config.provider)
    config.silero_vad.model = str(root / model["model"])
    config.silero_vad.window_size = VAD_WINDOW
    if "threshold" in model:
        config.silero_vad.threshold = model["threshold"]
    config.silero_vad.min_silence_duration = model.get("min_silence_duration", 0.4)
    config.silero_vad.min_speech_duration = model.get("min_speech_duration", 0.25)
    detector = sherpa.VoiceActivityDetector(config, 30.0)
    emit("vad_backend", provider=config.provider)
    return detector


def load_recognizer(root, config, *, enable_endpoints=False):
    started = time.perf_counter()
    provider = config.get("provider", "cpu")
    settings = dict(
        provider=provider,
        num_threads=speech_threads(config, provider),
        enable_endpoint_detection=enable_endpoints,
        rule1_min_trailing_silence=config.get("rule1_min_trailing_silence", 2.4),
        rule2_min_trailing_silence=config.get("rule2_min_trailing_silence", 1.2),
        rule3_min_utterance_length=config.get("rule3_min_utterance_length", 20.0),
    )
    if config["model_type"] == "zipformer2_ctc":
        recognizer = sherpa.OnlineRecognizer.from_zipformer2_ctc(
            **model_files(root, config, "tokens", "model"), **settings,
        )
    else:
        recognizer = sherpa.OnlineRecognizer.from_transducer(
            **model_files(root, config, "tokens", "encoder", "decoder", "joiner"),
            model_type=config["model_type"], **settings,
        )
    emit("asr_loaded", provider=provider, load_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib())
    return recognizer


def finish_recognition(recognizer, stream):
    stream.accept_waveform(SAMPLE_RATE, np.zeros(SAMPLE_RATE // 2, dtype=np.float32))
    stream.input_finished()
    while recognizer.is_ready(stream):
        recognizer.decode_stream(stream)
    return recognizer.get_result(stream).strip()


def load_synthesizer(root, config):
    started = time.perf_counter()
    if config["type"] == "vits-rknn":
        from rk3566_tts import RknnPiper

        engine = RknnPiper(root, config, speech_runtime(root))
        emit("tts_loaded", provider="rknn", encoder_provider="cpu",
             load_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib())
        return engine

    threads = config.get("threads", 2)
    rules = ""
    if config["type"] == "vits":
        voice = sherpa.OfflineTtsVitsModelConfig(**model_files(root, config, "model", "tokens"))
        network = sherpa.OfflineTtsModelConfig(vits=voice, num_threads=threads)
    elif config["type"] == "matcha":
        voice = sherpa.OfflineTtsMatchaModelConfig(
            **model_files(root, config, "acoustic_model", "vocoder", "tokens", "lexicon")
        )
        network = sherpa.OfflineTtsModelConfig(matcha=voice, num_threads=threads)
        rules = ",".join(str(root / path) for path in config["rule_fsts"])
    else:
        raise ValueError(f"Unknown TTS model type: {config['type']}")

    settings = sherpa.OfflineTtsConfig(model=network, rule_fsts=rules, max_num_sentences=1)
    if not settings.validate():
        raise ValueError("Invalid TTS model configuration")
    engine = sherpa.OfflineTts(settings)
    emit("tts_loaded", load_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib())
    return engine


def synthesize(engine, model_type, text, output):
    started = time.perf_counter()
    if model_type == "vits-rknn":
        audio = engine.generate(text)
    else:
        generation = sherpa.GenerationConfig()
        generation.num_steps = 3
        if model_type == "vits":
            from piper_phonemize import phonemize_espeak
            phonemes = phonemize_espeak(text, "cmn")
            generation.phoneme_codepoints = [[ord(char) for char in sentence] for sentence in phonemes]
        audio = engine.generate(text, generation)
    generation_ms = elapsed_ms(started)

    samples = np.asarray(audio.samples, dtype=np.float32)
    if not samples.size or not np.isfinite(samples).all() or not np.any(samples):
        raise RuntimeError("TTS returned empty or invalid audio")
    pcm = (np.clip(samples, -1, 1) * 32767).astype("<i2")
    with wave.open(str(output), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(audio.sample_rate)
        wav.writeframes(pcm.tobytes())
    duration = len(pcm) / audio.sample_rate
    emit("tts", text=text, synthesis_ms=generation_ms,
         audio_s=round(duration, 3), output=str(output))
    return duration


def play_audio(output, duration):
    player = SCRIPT_DIR / "audio-output.py"
    if not player.is_file() and SCRIPT_DIR.parent.name == "target":
        player = SCRIPT_DIR.parents[1] / "scripts/audio-output.py"
    if not player.is_file():
        raise RuntimeError(f"Robot speaker player is missing: {player}")
    # Reuse the robot's amplifier enable, warm-up and output lock while
    # PulseAudio shares the codec with microphone capture.
    subprocess.run(
        [str(player), "-q", "-D", "pulse", str(output.resolve())],
        check=True, timeout=duration + 10,
    )


def emit_ready(started, **fields):
    emit("ready", **fields, load_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib(),
         available_mib=available_memory_mib())


def emit_done(status, **fields):
    emit("done", status=status, **fields, peak_rss_mib=peak_rss_mib(),
         available_mib=available_memory_mib())


@contextmanager
def filter_rknn_stderr():
    # Sherpa calls rknn_set_core_mask for multi-core chips. The RK3566 uses
    # its single core automatically, yet this notice can repeat every frame.
    warning = (b"Failed to select npu core to run the model "
               b"(You can ignore it if you are not using RK3588.")
    with os.fdopen(os.dup(2), "wb", buffering=0) as stderr:
        read_fd, write_fd = os.pipe()

        def forward():
            with os.fdopen(read_fd, "rb") as reader:
                for line in reader:
                    if b"/csrc/rknn/utils.cc:SetCoreMask:" in line and line.rstrip().endswith(warning):
                        continue
                    stderr.write(line)

        worker = Thread(target=forward)
        worker.start()
        try:
            os.dup2(write_fd, 2)
        finally:
            os.close(write_fd)
        try:
            yield
        finally:
            os.dup2(stderr.fileno(), 2)
            worker.join()


def run_speech_test(main):
    with filter_rknn_stderr():
        try:
            main()
        except KeyboardInterrupt:
            emit_done("stopped")
