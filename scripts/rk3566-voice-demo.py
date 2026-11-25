#!/usr/bin/env python3
"""Run wake-up, transcription and a spoken reply using the RK3566 microphone."""

from rk3566_voice import run_speech_test
from rk3566_voice_pipeline import main


if __name__ == "__main__":
    run_speech_test(lambda: main(__doc__))
