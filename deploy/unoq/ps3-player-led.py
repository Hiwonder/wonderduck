#!/usr/bin/python3
"""Set player LED 1 on a Bluetooth Sixaxis handled by hid-generic."""

import fcntl
from pathlib import Path
import re
import sys


# Sixaxis output report 1. The rumble amplitudes are zero and bit 1 lights LED 1.
PLAYER_ONE = bytes.fromhex(
    "01 01 ff 00 ff 00 00 00 00 00 02 "
    "ff 27 10 00 32 ff 27 10 00 32 ff 27 10 00 32 ff 27 10 00 32 "
    "00 00 00 00 00"
)
# _IOWR('H', 0x0b, len), HIDIOCSOUTPUT on Linux arm64.
HIDIOCSOUTPUT = 0xC000480B | (len(PLAYER_ONE) << 16)


def main(name):
    if not re.fullmatch(r"hidraw[0-9]+", name):
        return
    device = Path("/sys/class/hidraw") / name / "device"
    try:
        properties = dict(line.split("=", 1) for line in (device / "uevent").read_text().splitlines())
        if properties.get("DRIVER") != "hid-generic" or properties.get("HID_ID") != "0005:0000054C:00000268":
            return
        with (Path("/dev") / name).open("r+b", buffering=0) as stream:
            if fcntl.ioctl(stream.fileno(), HIDIOCSOUTPUT, bytearray(PLAYER_ONE)) != len(PLAYER_ONE):
                raise OSError("short Sixaxis LED report")
    except OSError as error:
        sys.exit(f"PS3 player LED: {error}")


if __name__ == "__main__":
    main(sys.argv[1])
