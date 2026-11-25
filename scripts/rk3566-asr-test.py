#!/usr/bin/env python3
"""Transcribe Mandarin from the RK3566 microphone."""

import time

from rk3566_voice import (
    SAMPLE_RATE, add_microphone_arguments, emit, emit_done, emit_ready,
    finish_recognition, load_config, load_recognizer, open_microphone,
    run_speech_test, test_parser,
)


# 配置 / Settings
# None 继承 JSON 或程序默认值。相对模型路径以 JSON 所在目录为基准。
# None inherits JSON or program defaults. Model paths are relative to the JSON file.
SOURCE = None  # PulseAudio source, overridable with --source
SHOW_PARTIAL = False  # Overridable with --partial or --no-partial
MODEL_OVERRIDES = {
    "provider": None,  # rknn or cpu
    "model_type": None,  # zipformer or zipformer2_ctc
    "tokens": None,
    "encoder": None,  # Transducer models
    "decoder": None,
    "joiner": None,
    "model": None,  # zipformer2_ctc only
    "threads": None,  # CPU only, default 1
    "rule1_min_trailing_silence": None,  # Seconds without decoded tokens, default 2.4
    "rule2_min_trailing_silence": None,  # Seconds after decoded tokens, default 1.2
    "rule3_min_utterance_length": None,  # Seconds to force an endpoint, default 20.0
}


def main():
    parser = test_parser(__doc__)
    add_microphone_arguments(parser, source=SOURCE)
    partial = parser.add_mutually_exclusive_group()
    partial.add_argument("--partial", action="store_true", help="Show the transcript as it changes")
    partial.add_argument("--no-partial", action="store_false", dest="partial", help="Show final transcripts only")
    parser.set_defaults(partial=SHOW_PARTIAL)
    args = parser.parse_args()

    root, config = load_config(args.config, section="asr", overrides=MODEL_OVERRIDES)
    started = time.perf_counter()
    recognizer = load_recognizer(root, config["asr"], enable_endpoints=True)
    stream = recognizer.create_stream()
    samples_read = 0
    utterances = 0
    previous = ""
    try:
        with open_microphone(args, config) as (source, frames):
            emit_ready(started, source=source)
            for samples in frames:
                samples_read += len(samples)
                stream.accept_waveform(SAMPLE_RATE, samples)
                decoded = recognizer.is_ready(stream)
                while recognizer.is_ready(stream):
                    recognizer.decode_stream(stream)
                if not decoded:
                    continue

                audio_s = round(samples_read / SAMPLE_RATE, 3)
                if recognizer.is_endpoint(stream):
                    text = recognizer.get_result(stream).strip()
                    if text:
                        emit("asr", text=text, audio_s=audio_s)
                        utterances += 1
                    recognizer.reset(stream)
                    previous = ""
                elif args.partial:
                    text = recognizer.get_result(stream).strip()
                    if text and text != previous:
                        emit("partial", text=text, audio_s=audio_s)
                        previous = text
    except KeyboardInterrupt:
        text = finish_recognition(recognizer, stream) if samples_read else ""
        if text:
            emit("asr", text=text, audio_s=round(samples_read / SAMPLE_RATE, 3))
            utterances += 1

    emit_done("stopped", utterances=utterances, audio_s=round(samples_read / SAMPLE_RATE, 3))


if __name__ == "__main__":
    run_speech_test(main)
