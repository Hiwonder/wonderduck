#!/usr/bin/env python3
"""Wake up, recognize a Mandarin utterance, and speak the transcript verbatim."""

from rk3566_voice import run_speech_test
from rk3566_voice_pipeline import main


if __name__ == "__main__":
    run_speech_test(lambda: main(__doc__, echo=True))
