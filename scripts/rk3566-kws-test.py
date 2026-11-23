#!/usr/bin/env python3
"""Test Mandarin wake-word detection with the RK3566 microphone."""

import time

from rk3566_voice import (
    SAMPLE_RATE, add_microphone_arguments, detect_keyword, emit, emit_done, emit_ready,
    finish_spotting, load_config, load_spotter, open_microphone, run_microphone_test,
    test_parser,
)


def main():
    parser = test_parser(__doc__)
    add_microphone_arguments(parser)
    args = parser.parse_args()

    root, config = load_config(args.config)
    started = time.perf_counter()
    spotter = load_spotter(root, config["kws"])
    stream = spotter.create_stream()
    samples_read = 0
    hits = 0
    try:
        with open_microphone(args, config) as (source, frames):
            emit_ready(started, source=source)
            for samples in frames:
                samples_read += len(samples)
                stream.accept_waveform(SAMPLE_RATE, samples)
                keyword = detect_keyword(spotter, stream)
                if keyword:
                    hits += 1
                    emit("wake", keyword=keyword, detected_at_s=round(samples_read / SAMPLE_RATE, 3))
                    spotter.reset_stream(stream)
    except KeyboardInterrupt:
        if samples_read:
            keyword = finish_spotting(spotter, stream)
            if keyword:
                hits += 1
                emit("wake", keyword=keyword, detected_at_s=round(samples_read / SAMPLE_RATE, 3))

    emit_done("stopped", hits=hits, audio_s=round(samples_read / SAMPLE_RATE, 3))


if __name__ == "__main__":
    run_microphone_test(main)
