#!/usr/bin/env python3
"""Show where speech starts and ends in the RK3566 microphone input."""

import time

from rk3566_voice import (
    SAMPLE_RATE, add_microphone_arguments, emit_done, emit_ready, emit_speech,
    load_config, make_vad, open_microphone, run_microphone_test, test_parser,
)


def report_segments(vad):
    count = 0
    while not vad.empty():
        emit_speech(vad.front)
        vad.pop()
        count += 1
    return count


def main():
    parser = test_parser(__doc__)
    add_microphone_arguments(parser)
    args = parser.parse_args()

    root, config = load_config(args.config)
    started = time.perf_counter()
    vad = make_vad(root / config["vad"])
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
    run_microphone_test(main)
