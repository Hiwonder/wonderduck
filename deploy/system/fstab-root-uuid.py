#!/usr/bin/env python3
"""Replace an unresolved /dev/root fstab entry with the running filesystem UUID."""

import os
from pathlib import Path
import re
import stat
import subprocess
import tempfile


FSTAB = Path("/etc/fstab")


def rewrite_root(content, uuid):
    if not uuid or any(character.isspace() for character in uuid):
        raise ValueError("root filesystem UUID is invalid")
    lines = content.splitlines(keepends=True)
    entries = []
    for index, line in enumerate(lines):
        fields = line.split()
        if fields and not fields[0].startswith("#") and len(fields) >= 2 and fields[1] == "/":
            entries.append((index, fields[0]))
    if len(entries) != 1:
        raise ValueError("fstab must contain exactly one root filesystem entry")
    index, source = entries[0]
    expected = f"UUID={uuid}"
    if source not in ("/dev/root", expected):
        raise ValueError("root fstab source is not the expected /dev/root")
    match = re.match(r"^(\s*)\S+(\s+/\s+\S+\s+)(\S+)(.*)", lines[index])
    if not match:
        raise ValueError("root fstab entry has no mount options")
    options = [option for option in match.group(3).split(",") if option != "noauto"]
    lines[index] = (match.group(1) + expected + match.group(2) + ",".join(options or ["defaults"])
                    + match.group(4) + lines[index][match.end():])
    return "".join(lines)


def command(*args):
    return subprocess.check_output(args, text=True).strip()


def main():
    if FSTAB.is_symlink():
        raise RuntimeError("fstab must be a regular file")
    root = command("findmnt", "--raw", "--noheadings", "--mountpoint", "/", "--output", "SOURCE")
    if not stat.S_ISBLK(os.stat(root).st_mode):
        raise RuntimeError("the running root source is not a block device")
    uuid = command("blkid", "--probe", "--output", "value", "--match-tag", "UUID", root)
    matches = command("blkid", "--match-token", f"UUID={uuid}", "--output", "device").splitlines()
    if len(matches) != 1 or os.path.realpath(matches[0]) != os.path.realpath(root):
        raise RuntimeError("the root filesystem UUID does not resolve uniquely")
    original = FSTAB.read_text(encoding="utf-8")
    updated = rewrite_root(original, uuid)
    if updated == original:
        print("root fstab already identifies the running filesystem")
        return
    descriptor, name = tempfile.mkstemp(prefix=".fstab-", dir=str(FSTAB.parent))
    candidate = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            output.write(updated)
            output.flush()
            os.fchmod(output.fileno(), 0o644)
            os.fchown(output.fileno(), 0, 0)
            os.fsync(output.fileno())
        subprocess.run(["findmnt", "--verify", "--tab-file", str(candidate)], check=True,
                       stdout=subprocess.DEVNULL)
        resolved = command("findmnt", "--fstab", "--evaluate", "--tab-file", str(candidate),
                           "--noheadings", "--mountpoint", "/", "--output", "SOURCE")
        if os.path.realpath(resolved) != os.path.realpath(root):
            raise RuntimeError("candidate fstab does not resolve to the running root")
        os.replace(candidate, FSTAB)
        directory = os.open(str(FSTAB.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        candidate.unlink(missing_ok=True)
    print("root fstab now identifies the running filesystem")


if __name__ == "__main__":
    main()
