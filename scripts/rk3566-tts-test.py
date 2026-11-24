#!/usr/bin/env python3
"""Synthesize Chinese text to a WAV and optionally play it on RK3566."""

from pathlib import Path
import time

from rk3566_voice import (
    emit_done, emit_ready, load_config, load_synthesizer, play_audio, synthesize, test_parser,
)


def main():
    parser = test_parser(__doc__)
    parser.add_argument("--text", required=True, help="Chinese text to synthesize")
    parser.add_argument("--output", type=Path, default=Path("tts-test.wav"))
    parser.add_argument("--play", action="store_true", help="Play the resulting WAV")
    args = parser.parse_args()
    text = args.text.strip()
    if not text:
        parser.error("--text must contain text to synthesize")

    root, config = load_config(args.config)
    started = time.perf_counter()
    engine = load_synthesizer(root, config["tts"])
    emit_ready(started)
    duration = synthesize(engine, config["tts"]["type"], text, args.output)
    if args.play:
        play_audio(args.output, duration)
    emit_done("complete")


if __name__ == "__main__":
    main()
