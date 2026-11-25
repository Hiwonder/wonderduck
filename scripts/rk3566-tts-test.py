#!/usr/bin/env python3
"""Synthesize Chinese text to a WAV and optionally play it on RK3566."""

from pathlib import Path
import time

from rk3566_voice import (
    emit_done, emit_ready, load_config, load_synthesizer, play_audio, run_speech_test, synthesize, test_parser,
)


# 配置 / Settings
# None 继承 JSON 或程序默认值。相对模型路径以 JSON 所在目录为基准。
# None inherits JSON or program defaults. Model paths are relative to the JSON file.
TEXT = "你好，欢迎使用小鸭机器人。"
OUTPUT = Path("tts-test.wav")
PLAY = False  # Overridable with --play or --no-play
MODEL_OVERRIDES = {
    "type": None,  # vits-rknn, vits or matcha
    "encoder": None,  # vits-rknn encoder (.onnx)
    "decoder": None,  # vits-rknn decoder (.rknn)
    "phoneme_config": None,  # vits-rknn phoneme configuration (.json)
    "model": None,  # CPU vits model (.onnx)
    "tokens": None,  # CPU vits or matcha
    "acoustic_model": None,  # CPU matcha model (.onnx)
    "vocoder": None,
    "lexicon": None,
    "rule_fsts": None,  # A list of Matcha rule file paths
    "threads": None,  # CPU threads, default 2
}


def main():
    parser = test_parser(__doc__)
    parser.add_argument("--text", default=TEXT, help="Chinese text to synthesize")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    playback = parser.add_mutually_exclusive_group()
    playback.add_argument("--play", action="store_true", help="Play the resulting WAV through the robot speaker")
    playback.add_argument("--no-play", action="store_false", dest="play", help="Write the WAV without playing it")
    parser.set_defaults(play=PLAY)
    args = parser.parse_args()
    text = args.text.strip()
    if not text:
        parser.error("--text must contain text to synthesize")

    root, config = load_config(args.config, section="tts", overrides=MODEL_OVERRIDES)
    started = time.perf_counter()
    engine = load_synthesizer(root, config["tts"])
    emit_ready(started)
    duration = synthesize(engine, config["tts"]["type"], text, args.output)
    if args.play:
        play_audio(args.output, duration)
    emit_done("complete")


if __name__ == "__main__":
    run_speech_test(main)
