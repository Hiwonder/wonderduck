#!/bin/sh
set -eu

board=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$board/../.." && pwd)
config=/etc/robot

if [ "$(id -u)" -ne 0 ] || [ "$root" != /home/arduino/wonderduck ]; then
    echo "Run as root from /home/arduino/wonderduck/deploy/unoq." >&2
    exit 1
fi

test -x "$root/target/release/robotd"
test -r "$board/robotd.toml"
test -r "$board/robotd.models.json"
if systemctl is-active --quiet robotd.service; then
    echo "robotd.service is active." >&2
    exit 1
fi
visudo -cf "$board/wonderduck-unoq.sudoers"

if [ ! -d "$config" ]; then
    install -d -m 0755 "$config"
fi
if [ ! -e "$config/robotd.toml" ]; then
    (
        umask 027
        sed 's@^port = "/dev/wonderduck-bus"$@port = "/run/wonderduck-unoq/bus"@' \
            "$board/robotd.toml" > "$config/robotd.toml"
    )
    chgrp arduino "$config/robotd.toml"
fi
for source in "$board"/robotd.*.json "$board/device-version.json"; do
    [ -f "$source" ] || continue
    target="$config/${source##*/}"
    if [ ! -e "$target" ]; then
        install -m 0644 "$source" "$target"
    fi
done

install -m 0440 "$board/wonderduck-unoq.sudoers" /etc/sudoers.d/wonderduck-unoq
install -m 0644 "$board/wonderduck-unoq-bridge.service" "$board/robotd.service" /etc/systemd/system/
systemctl daemon-reload
