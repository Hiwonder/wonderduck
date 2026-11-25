#!/usr/bin/env python3
"""Test Mandarin wake-word detection with the RK3566 microphone."""

import time

from rk3566_voice import (
    SAMPLE_RATE, add_microphone_arguments, detect_keyword, emit, emit_done, emit_ready,
    finish_spotting, load_config, load_spotter, open_microphone, run_speech_test,
    test_parser,
)


# 配置 / Settings
# None 继承 JSON 或程序默认值。相对模型路径以 JSON 所在目录为基准。
# None inherits JSON or program defaults. Model paths are relative to the JSON file.
SOURCE = None  # PulseAudio source, overridable with --source
MODEL_OVERRIDES = {
    "provider": None,  # rknn or cpu
    "encoder": None,
    "decoder": None,
    "joiner": None,
    "tokens": None,
    "keywords_file": None,
    "threshold": None,  # Default 0.25
    "score": None,  # Default 1.0
    "trailing_blanks": None,  # Default 1
    "threads": None,  # CPU only, default 1
}
# Set a Sherpa-ONNX entry per line to use inline keywords in place of keywords_file.
# 示例 / Example: KEYWORDS = "x iǎo y ā x iǎo y ā @小鸭小鸭"
KEYWORDS = None


def main():
    parser = test_parser(__doc__)
    add_microphone_arguments(parser, source=SOURCE)
    args = parser.parse_args()

    root, config = load_config(args.config, section="kws", overrides=MODEL_OVERRIDES)
    started = time.perf_counter()
    spotter = load_spotter(root, config["kws"], keywords=KEYWORDS)
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
    run_speech_test(main)
