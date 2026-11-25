#!/usr/bin/env python3
"""Show where speech starts and ends in the RK3566 microphone input."""

import time

from rk3566_voice import (
    SAMPLE_RATE, add_microphone_arguments, emit_done, emit_ready, emit_speech,
    load_config, make_vad, open_microphone, run_speech_test, test_parser,
)


# 配置 / Settings
# None 继承 JSON 或程序默认值。相对模型路径以 JSON 所在目录为基准。
# None inherits JSON or program defaults. Model paths are relative to the JSON file.
SOURCE = None  # PulseAudio source, overridable with --source
MODEL_OVERRIDES = {
    "provider": None,  # rknn or cpu
    "model": None,  # Silero VAD, 16 kHz
    "threshold": None,  # Speech probability, 0 to 1
    "min_silence_duration": None,  # Seconds, default 0.4
    "min_speech_duration": None,  # Seconds, default 0.25
    "threads": None,  # CPU only, default 1
}


def report_segments(vad):
    count = 0
    while not vad.empty():
        emit_speech(vad.front)
        vad.pop()
        count += 1
    return count


def main():
    parser = test_parser(__doc__)
    add_microphone_arguments(parser, source=SOURCE)
    args = parser.parse_args()

    root, config = load_config(args.config, section="vad", overrides=MODEL_OVERRIDES)
    started = time.perf_counter()
    vad = make_vad(root, config["vad"])
    samples_read = 0
    segments = 0
    try:
        with open_microphone(args, config) as (source, frames):
            emit_ready(started, source=source)
            for samples in frames:
                samples_read += len(samples)
                vad.accept_waveform(samples)
                segments += report_segments(vad)
    except KeyboardInterrupt:
        vad.flush()
        segments += report_segments(vad)

    emit_done("stopped", segments=segments, audio_s=round(samples_read / SAMPLE_RATE, 3))


if __name__ == "__main__":
    run_speech_test(main)
