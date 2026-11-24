#!/usr/bin/env python3
"""Run wake-up, transcription and a spoken reply using the RK3566 microphone."""

from pathlib import Path
import time

from rk3566_voice import (
    SAMPLE_RATE, add_microphone_arguments, detect_keyword, elapsed_ms, emit, emit_done,
    emit_ready, emit_speech, finish_recognition, load_config, load_recognizer,
    load_spotter, load_synthesizer, make_vad, open_microphone, peak_rss_mib,
    play_audio, run_microphone_test, synthesize, test_parser,
)


def listen(frames, spotter, vad):
    stream = spotter.create_stream()
    vad.reset()
    samples_read = 0
    wake_at = None

    for samples in frames:
        samples_read += len(samples)
        if wake_at is None:
            stream.accept_waveform(SAMPLE_RATE, samples)
            keyword = detect_keyword(spotter, stream)
            if keyword:
                wake_at = samples_read
                emit("wake", keyword=keyword, detected_at_s=round(wake_at / SAMPLE_RATE, 3))
        else:
            vad.accept_waveform(samples)
            if not vad.empty():
                return emit_speech(vad.front, offset=wake_at)


def transcribe(recognizer, samples):
    started = time.perf_counter()
    stream = recognizer.create_stream()
    stream.accept_waveform(SAMPLE_RATE, samples)
    text = finish_recognition(recognizer, stream)
    emit("asr", text=text, decode_ms=elapsed_ms(started), peak_rss_mib=peak_rss_mib())
    return text


def main():
    parser = test_parser(__doc__)
    add_microphone_arguments(parser)
    parser.add_argument("--output", type=Path, default=Path("voice-reply.wav"))
    parser.add_argument("--play", action="store_true", help="Play the reply through PulseAudio")
    parser.add_argument("--warm", action="store_true", help="Load ASR and TTS before listening")
    args = parser.parse_args()

    root, config = load_config(args.config)
    started = time.perf_counter()
    spotter = load_spotter(root, config["kws"])
    vad = make_vad(root / config["vad"])
    recognizer = load_recognizer(root, config["asr"]) if args.warm else None
    synthesizer = load_synthesizer(root, config["tts"]) if args.warm else None
    completed = 0
    first_capture = True

    try:
        while True:
            with open_microphone(args, config) as (source, frames):
                if first_capture:
                    emit_ready(started, source=source, preload=args.warm)
                    first_capture = False
                else:
                    emit("listening", source=source)
                samples = listen(frames, spotter, vad)

            started_reply = time.perf_counter()
            if recognizer is None:
                recognizer = load_recognizer(root, config["asr"])
            text = transcribe(recognizer, samples)
            if not text:
                continue
            if synthesizer is None:
                synthesizer = load_synthesizer(root, config["tts"])
            duration = synthesize(
                synthesizer, config["tts"]["type"], "我听到了。您说的是，" + text + "。", args.output,
            )
            emit("reply_ready", after_speech_ms=elapsed_ms(started_reply))
            if args.play:
                play_audio(args.output, duration)
            completed += 1
    except KeyboardInterrupt:
        emit_done("stopped", replies=completed)


if __name__ == "__main__":
    run_microphone_test(main)
