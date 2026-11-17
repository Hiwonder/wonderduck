"""Complete BlueZ HID registration for recognized incoming PS3 controllers."""

import configparser
import io
import logging
import os
from pathlib import Path
import re
import subprocess
import tempfile


HID_UUID = "00001124-0000-1000-8000-00805f9b34fb"
NAMES = {"Sony PLAYSTATION(R)3 Controller", "SHANWAN PS3 GamePad", "PS3 Controller"}
ADDRESS = re.compile(r"(?:[0-9A-F]{2}:){5}[0-9A-F]{2}\Z")
# BlueZ 5.53 plugins/sixaxis.c, SIXAXIS_HID_SDP_RECORD (405 bytes).
HID_RECORD = (
    "3601920900000A000100000900013503191124090004350D35061901000900113503190011090006350909656E09006A090100090009"
    "3508350619112409010009000D350F350D350619010009001335031900110901002513576972656C65737320436F6E74726F6C6C65"
    "720901012513576972656C65737320436F6E74726F6C6C6572090102251B536F6E7920436F6D707574657220456E746572746169"
    "6E6D656E740902000901000902010901000902020800090203082109020428010902052801090206359A35980822259405010904A101"
    "A102850175089501150026FF00810375019513150025013500450105091901291381027501950D0600FF8103150026FF0005010901"
    "A10075089504350046FF0009300931093209358102C0050175089527090181027508953009019102750895300901B102C0A1028502"
    "750895300901B102C0A10285EE750895300901B102C0A10285EF750895300901B102C0C00902073508350609040909010009020828"
    "00090209280109020A280109020B09010009020C093E8009020D280009020E2800"
)


def read_config(path):
    config = configparser.ConfigParser(interpolation=None)
    config.optionxform = str
    if path.exists():
        with path.open(encoding="utf-8") as stream:
            config.read_file(stream)
    return config


def needs_registration(base, address, discovered_name=None):
    if not ADDRESS.fullmatch(address):
        return False
    cache = read_config(base / "cache" / address)
    name = cache.get("General", "Name", fallback="") or discovered_name
    if name not in NAMES:
        return False
    info = read_config(base / address / "info")
    if info.getboolean("General", "Blocked", fallback=False):
        return False
    stored_name = info.get("General", "Name", fallback=name)
    if stored_name not in NAMES:
        logging.warning("PS3 registration skipped: existing device name differs")
        return False
    services = info.get("General", "Services", fallback="").lower().split(";")
    records = dict(cache.items("ServiceRecords")) if cache.has_section("ServiceRecords") else {}
    if HID_UUID in services and any("191124" in value.upper() for value in records.values()):
        return False
    if records.get("0x00010000", HID_RECORD).upper() != HID_RECORD:
        logging.warning("PS3 registration skipped: service record handle is already in use")
        return False
    for key, value in (("Source", "2"), ("Vendor", "1356"), ("Product", "616")):
        if info.has_option("DeviceID", key) and info.get("DeviceID", key) != value:
            logging.warning("PS3 registration skipped: existing device identity differs")
            return False
    return True


def candidates(root, adapter_addresses, discovered=()):
    found = []
    adapters = {address for address in adapter_addresses if ADDRESS.fullmatch(address)}
    for adapter in sorted(adapters):
        base = root / adapter
        cache = base / "cache"
        if not cache.is_dir():
            continue
        for path in sorted(cache.iterdir()):
            if path.is_file() and needs_registration(base, path.name):
                found.append((base, path.name))
    known = set(found)
    for adapter, address, name in discovered:
        base = root / adapter
        candidate = (base, address)
        if (adapter in adapters and name in NAMES and candidate not in known
                and needs_registration(base, address, name)):
            found.append(candidate)
            known.add(candidate)
    return found


def write_config(path, config):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    output = io.StringIO()
    config.write(output, space_around_delimiters=False)
    fd, temporary = tempfile.mkstemp(prefix=".sixaxis-", dir=str(path.parent))
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(output.getvalue())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def register(base, address, discovered_name=None):
    if not needs_registration(base, address, discovered_name):
        return False
    cache_path = base / "cache" / address
    info_path = base / address / "info"
    cache = read_config(cache_path)
    info = read_config(info_path)
    if not cache.has_section("General"):
        cache.add_section("General")
    if not cache.get("General", "Name", fallback=""):
        cache["General"]["Name"] = discovered_name
    if not info.has_section("General"):
        info.add_section("General")
    info["General"]["Name"] = cache["General"]["Name"]
    info["General"]["Trusted"] = "true"
    services = [value for value in info.get("General", "Services", fallback="").split(";") if value]
    if HID_UUID not in (value.lower() for value in services):
        services.append(HID_UUID)
    info["General"]["Services"] = ";".join(services) + ";"
    if not info.has_section("DeviceID"):
        info.add_section("DeviceID")
    info["DeviceID"].update({"Source": "2", "Vendor": "1356", "Product": "616", "Version": "0"})
    if not cache.has_section("ServiceRecords"):
        cache.add_section("ServiceRecords")
    cache["ServiceRecords"]["0x00010000"] = HID_RECORD
    write_config(cache_path, cache)
    write_config(info_path, info)
    return True


def active_input_addresses():
    devices = Path("/proc/bus/input/devices").read_text(encoding="utf-8")
    addresses = set()
    for device in devices.split("\n\n"):
        if not re.search(r"(?m)^I: Bus=0005\b", device):
            continue
        address = re.search(r"(?m)^U: Uniq=([^\s]+)", device)
        if address:
            addresses.add(address.group(1).upper())
    return addresses


def register_pending(root, adapter_addresses, connected, discovered=()):
    adapter_addresses = list(adapter_addresses)
    discovered = list(discovered)
    pending = candidates(root, adapter_addresses, discovered)
    if not pending:
        return False
    pending_keys = {(base.name, address) for base, address in pending}
    active_inputs = active_input_addresses() if pending_keys.intersection(connected) else set()
    if any(pair not in pending_keys or pair[1] in active_inputs for pair in connected):
        return False
    try:
        subprocess.run(["systemctl", "stop", "bluetooth.service"], check=True, timeout=30)
        names = {(adapter, address): name for adapter, address, name in discovered if name in NAMES}
        count = sum(register(base, address, names.get((base.name, address))) for base, address in pending)
    finally:
        subprocess.run(["systemctl", "start", "bluetooth.service"], check=True, timeout=30)
    if count:
        logging.info("Registered %d PS3 controller(s); press PS to reconnect", count)
    return bool(count)
