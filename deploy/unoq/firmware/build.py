#!/usr/bin/env python3
"""Build the UNO Q bridge with bounded RouterBridge framing and 2 Mbaud UART."""

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def replace_once(path, old, new):
    source = path.read_text()
    if source.count(old) != 1:
        raise RuntimeError(f"unexpected library source in {path}: {old!r}")
    path.write_text(source.replace(old, new, 1))


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--libraries", type=Path, default=Path.home() / "Arduino/libraries")
    parser.add_argument("--build-path", type=Path, default=here / "build")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="wonderduck-unoq-libs-") as temporary:
        libs = Path(temporary)
        for name in ("Arduino_RouterBridge", "Arduino_RPClite"):
            shutil.copytree(args.libraries / name, libs / name)

        bridge = libs / "Arduino_RouterBridge/src/bridge.h"
        source = bridge.read_text()
        source, stack_count = re.subn(
            r"(#define\s+UPDATE_THREAD_STACK_SIZE\s+)\d+", r"\g<1>4096", source, count=1)
        if stack_count != 1:
            raise RuntimeError("RouterBridge update thread stack setting not found")
        bridge.write_text(source)
        replace_once(bridge, "        serial_ptr->begin(baud);",
                     "        // The sketch configured the router UART and its FIFO before Bridge.begin().")

        rpclite = libs / "Arduino_RPClite/src"
        header = rpclite / "Arduino_RPClite.h"
        source = header.read_text()
        source, buffer_count = re.subn(
            r"(#define\s+DECODER_BUFFER_SIZE\s+)\d+", r"\g<1>4096", source, count=1)
        if buffer_count != 1:
            raise RuntimeError("RPClite decoder buffer setting not found")
        header.write_text(source)

        decoder = rpclite / "decoder.h"
        source = decoder.read_text()
        start = source.index("    void parse_packet(){")
        end = source.index("    bool packet_incoming()", start)
        source = source[:start] + (here / "rpclite/parse_packet.txt").read_text() + source[end:]
        include = '#include "transport.h"'
        if source.count(include) != 1:
            raise RuntimeError("RPClite transport include not found")
        decoder.write_text(source.replace(include, include + '\n#include "duck_msgpack_frame.h"', 1))
        shutil.copyfile(here / "rpclite/duck_msgpack_frame.h", rpclite / "duck_msgpack_frame.h")

        flags = f"-DWONDERDUCK_BRIDGE_BAUD=2000000 -include {here / 'bridge_buffers.h'}"
        subprocess.run([
            "arduino-cli", "compile", "--jobs", str(args.jobs), "--fqbn", "arduino:zephyr:unoq",
            "--library", str(libs / "Arduino_RouterBridge"),
            "--library", str(libs / "Arduino_RPClite"),
            "--build-path", str(args.build_path.resolve()),
            "--build-property", f"compiler.cpp.extra_flags={flags}",
            str(here / "wonderduck_bridge"),
        ], check=True)


if __name__ == "__main__":
    main()
