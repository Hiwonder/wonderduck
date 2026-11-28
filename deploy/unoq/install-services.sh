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
        sed -e 's@^port = "/dev/wonderduck-bus"$@port = "/run/wonderduck-unoq/bus"@' \
            -e '/^\[audio\]/,/^\[/s@^device = "default"$@device = "plughw:ArduinoImolaHPH,1"\ncapture_device = "plughw:ArduinoImolaHPH,2"@' \
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

actions="$root/extensions/wonderduck_studio/console/action-groups"
if [ ! -d "$actions" ]; then
    install -d -m 0700 -o arduino -g arduino "$actions"
fi
if [ ! -e "$actions/rest.csv" ]; then
    install -m 0600 -o arduino -g arduino "$board/rest.csv" "$actions/rest.csv"
fi

install -m 0440 "$board/wonderduck-unoq.sudoers" /etc/sudoers.d/wonderduck-unoq
install -m 0644 "$board/wonderduck-unoq-bridge.service" "$board/robotd.service" /etc/systemd/system/
install -d -m 0755 /usr/local/libexec
install -m 0644 "$board/recover-audio.sh" /usr/local/libexec/wonderduck-unoq-audio-recover
install -m 0644 "$board/wonderduck-unoq-audio-recover.service" /etc/systemd/system/
"$board/install-wifi-manager.sh"
install -d -m 0755 /etc/systemd/system/arduino-router.service.d
install -m 0644 "$board/arduino-router-affinity.conf" /etc/systemd/system/arduino-router.service.d/70-wonderduck-affinity.conf

# Sixaxis uses USB-programmed host addresses and reconnects without an SSP bond.
if [ -f /etc/bluetooth/input.conf ] && command -v bluetoothctl >/dev/null 2>&1; then
    python3 -c 'import dbus; from gi.repository import GLib'
    install -d -m 0755 /usr/local/libexec /etc/udev/rules.d
    install -m 0755 "$root/scripts/bluetooth_auto_trust.py" /usr/local/libexec/robot-bluetooth-auto-trust.py
    install -m 0644 "$root/scripts/bluetooth_sixaxis.py" /usr/local/libexec/
    install -m 0644 "$board/ps3-player-led.py" /usr/local/libexec/wonderduck-ps3-player-led.py
    install -m 0644 "$board/99-wonderduck-ps3-led.rules" "$root/deploy/input/99-gamepad-pairing.rules" /etc/udev/rules.d/
    install -m 0644 "$root/deploy/systemd/bluetooth-auto-trust.service" /etc/systemd/system/
    input=/etc/bluetooth/input.conf
    if ! grep -Eq '^[[:space:]]*ClassicBondedOnly[[:space:]]*=[[:space:]]*false[[:space:]]*$' "$input"; then
        if grep -Eq '^[[:space:]]*#?[[:space:]]*ClassicBondedOnly[[:space:]]*=' "$input"; then
            sed -i -E 's/^[[:space:]]*#?[[:space:]]*ClassicBondedOnly[[:space:]]*=.*/ClassicBondedOnly=false/' "$input"
        else
            sed -i '/^\[General\]/a ClassicBondedOnly=false' "$input"
        fi
        systemctl restart bluetooth.service
    fi
    udevadm control --reload-rules
    systemctl enable --now bluetooth-auto-trust.service
fi
systemctl daemon-reload
systemctl enable --now wonderduck-unoq-audio-recover.service
