#!/bin/sh
set -eu

board=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_dir=${1:-"$board/../../extensions/network"}

/usr/bin/python3 -c 'import dbus'
test -x /usr/sbin/dnsmasq
install -d -m 0755 /opt/robot/wifi-manager
install -m 0644 "$source_dir/wifi_manager.py" "$source_dir/wifi_network.py" /opt/robot/wifi-manager/
install -m 0644 "$source_dir/wifi-manager.service" /etc/systemd/system/
install -d -m 0755 /etc/systemd/system/wifi-manager.service.d
install -m 0644 "$board/wifi-hotspot-band.conf" /etc/systemd/system/wifi-manager.service.d/70-wonderduck-unoq.conf

if [ ! -e /etc/wifi-manager/hotspot.psk ]; then
    install -d -m 0700 /etc/wifi-manager
    (umask 077; printf '%s\n' hiwonder > /etc/wifi-manager/hotspot.psk)
fi

systemctl daemon-reload
if systemctl cat wonderduck-unoq-discovery.service >/dev/null 2>&1; then
    systemctl disable --now wonderduck-unoq-discovery.service
fi
systemctl enable wifi-manager.service
systemctl restart wifi-manager.service
