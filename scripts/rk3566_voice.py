"""Audio input and model helpers for the RK3566 speech tests."""

import argparse
from contextlib import contextmanager
from itertools import chain
import json
from pathlib import Path
import resource
import subprocess
import sys
import time
import wave

SCRIPT_DIR = Path(__file__).resolve().parent
# The RK3566 deployment keeps the ARM64 wheels beside the test programs.
for dependency in (SCRIPT_DIR / "runtime/python", SCRIPT_DIR / "tts/frontend"):
    if dependency.is_dir():
        sys.path.insert(0, str(dependency))

import numpy as np
import sherpa_onnx as sherpa


SAMPLE_RATE = 16000
VAD_WINDOW = 512


def test_parser(description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", type=Path, default=SCRIPT_DIR / "rk3566-voice-demo.json")
    return parser


def add_microphone_arguments(parser):
    parser.add_argument("--source", help="Override the configured PulseAudio microphone")
    parser.epilog = "Press Ctrl+C to stop listening."


def load_config(path):
    return path.resolve().parent, json.loads(path.read_text(encoding="utf-8"))


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


def load_spotter(root, config):
    return sherpa.KeywordSpotter(
        **model_files(root, config, "tokens", "encoder", "decoder", "joiner", "keywords_file"),
        num_threads=1, keywords_threshold=config.get("threshold", 0.25),
        keywords_score=config.get("score", 1.0), num_trailing_blanks=config.get("trailing_blanks", 1),
    )


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


def make_vad(path):
    config = sherpa.VadModelConfig()
    config.sample_rate = SAMPLE_RATE
    config.num_threads = 1
    config.silero_vad.model = str(path)
    config.silero_vad.window_size = VAD_WINDOW
    config.silero_vad.min_silence_duration = 0.4
    config.silero_vad.min_speech_duration = 0.25
    return sherpa.VoiceActivityDetector(config, 30.0)


def load_recognizer(root, config, *, enable_endpoints=False):
    started = time.perf_counter()
    threads = config.get("threads", 1)
    if config["model_type"] == "zipformer2_ctc":
        recognizer = sherpa.OnlineRecognizer.from_zipformer2_ctc(
            **model_files(root, config, "tokens", "model"), num_threads=threads,
            enable_endpoint_detection=enable_endpoints,
        )
    else:
        recognizer = sherpa.OnlineRecognizer.from_transducer(
            **model_files(root, config, "tokens", "encoder", "decoder", "joiner"),
            model_type=config["model_type"], num_threads=threads,
            enable_endpoint_detection=enable_endpoints,
        )
    emit("asr_loaded", load_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib())
    return recognizer


def finish_recognition(recognizer, stream):
    stream.accept_waveform(SAMPLE_RATE, np.zeros(SAMPLE_RATE // 2, dtype=np.float32))
    stream.input_finished()
    while recognizer.is_ready(stream):
        recognizer.decode_stream(stream)
    return recognizer.get_result(stream).strip()


def load_synthesizer(root, config):
    started = time.perf_counter()
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
    subprocess.run(
        ["paplay", "--client-name=WonderDuck voice test", str(output)],
        check=True, timeout=duration + 10,
    )


def emit_ready(started, **fields):
    emit("ready", **fields, load_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib(),
         available_mib=available_memory_mib())


def emit_done(status, **fields):
    emit("done", status=status, **fields, peak_rss_mib=peak_rss_mib(),
         available_mib=available_memory_mib())


def run_microphone_test(main):
    try:
        main()
    except KeyboardInterrupt:
        emit_done("stopped")
