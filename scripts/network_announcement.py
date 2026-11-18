"""Prepare short, offline spoken network announcements for the prompt service."""

import io
import ipaddress
import json
import os
from pathlib import Path
import queue
import re
import socket
import stat
import subprocess
import sys
import threading
import time
import wave


ROOT = Path(__file__).resolve().parent.parent
CLIPS = ROOT / "assets/prompt-sounds/network"
SOCKET = Path("/run/wonderduck-network-speech.sock")
MARKER = Path("/run/wonderduck-network-speech.played")
INTERFACE = "wlan0"
MAX_PENDING = 8
CHARACTER_PAUSE_FRAMES = 4320  # 180 ms at 24 kHz
SEPARATOR_PAUSE_FRAMES = 14400  # 600 ms at 24 kHz


def _run(*arguments):
    result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, timeout=3, env=dict(os.environ, LC_ALL="C"))
    if result.returncode:
        raise RuntimeError("network information is unavailable")
    return result.stdout.strip()


def address():
    data = json.loads(_run("ip", "-j", "-4", "address", "show", "dev", INTERFACE))
    for device in data:
        for item in device.get("addr_info", []):
            if item.get("family") != "inet" or item.get("scope") != "global":
                continue
            value = ipaddress.IPv4Address(item["local"])
            if not value.is_link_local and not value.is_loopback:
                return str(value)
    raise RuntimeError("wireless IP address is unavailable")


def active_network():
    values = _run("nmcli", "-g", "GENERAL.STATE,GENERAL.CON-UUID",
                  "device", "show", INTERFACE).splitlines()
    if len(values) != 2 or not values[0].startswith("100 ") or not values[1]:
        raise RuntimeError("wireless network is not connected")
    uuid = values[1]
    mode = _run("nmcli", "-g", "802-11-wireless.mode", "connection", "show", "uuid", uuid)
    if mode == "ap":
        name = _run("nmcli", "-e", "no", "-g", "802-11-wireless.ssid",
                    "connection", "show", "uuid", uuid)
        if not name:
            raise RuntimeError("hotspot name is unavailable")
        return "hotspot", name
    if mode in ("infrastructure", "", "--"):
        return "ip", address()
    raise RuntimeError("wireless mode is unavailable")


def hotspot_name():
    try:
        kind, value = active_network()
        if kind == "hotspot":
            return value
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError):
        pass
    # The device's Wi-Fi manager exposes the same identity used in its WD- SSID.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(2)
        client.sendto(b"LOBOT_NET_DISCOVER", ("127.0.0.1", 9027))
        response = client.recv(128).decode("ascii").strip()
    match = re.fullmatch(r"WonderDuck:([A-Za-z0-9]{1,32})", response)
    if not match:
        raise RuntimeError("hotspot name is unavailable")
    return "WD-" + match.group(1)


def tokens(kind, value):
    if kind not in ("hotspot", "ip") or not isinstance(value, str) or not 1 <= len(value) <= 32:
        raise ValueError("invalid network announcement")
    if kind == "ip":
        value = str(ipaddress.IPv4Address(value))
    elif value[:3].lower() == "wd-":
        value = value[3:]
        if not value:
            raise ValueError("invalid network announcement")
    result = []
    for character in value.lower():
        if character in "abcdefghijklmnopqrstuvwxyz0123456789":
            result.append(character)
        elif character in ".-":
            result.append(None)
        else:
            raise ValueError("network name contains unsupported characters")
    return result


def render(kind, value, clips=CLIPS):
    output = io.BytesIO()
    with wave.open(output, "wb") as combined:
        combined.setnchannels(1)
        combined.setsampwidth(2)
        combined.setframerate(24000)
        for name in tokens(kind, value):
            if name is None:
                combined.writeframes(b"\x00\x00" * SEPARATOR_PAUSE_FRAMES)
                continue
            with wave.open(str(clips / (name + ".wav")), "rb") as clip:
                if ((clip.getnchannels(), clip.getsampwidth(), clip.getframerate()) != (1, 2, 24000)
                        or not 0 < clip.getnframes() <= 96000):
                    raise ValueError("invalid network audio clip")
                combined.writeframes(clip.readframes(clip.getnframes()))
            combined.writeframes(b"\x00\x00" * CHARACTER_PAUSE_FRAMES)
    return output.getvalue()


class NetworkAnnouncements:
    def __init__(self, clips=CLIPS, marker=MARKER, socket_path=SOCKET):
        self.clips = clips
        self.marker = marker
        self.socket_path = socket_path
        self.requests = queue.Queue(maxsize=MAX_PENDING)
        self.ready = queue.Queue(maxsize=MAX_PENDING)
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.listener = threading.Thread(target=self._listen, daemon=True)

    def start(self):
        self.worker.start()
        self.listener.start()

    def stop(self):
        self.stopping.set()
        self.listener.join(timeout=2)
        self.worker.join(timeout=4)

    def _put_ready(self, item):
        while not self.stopping.is_set():
            try:
                self.ready.put(item, timeout=0.5)
                return
            except queue.Full:
                continue

    def _work(self):
        automatic = not self.marker.exists()
        previous = None
        while not self.stopping.is_set():
            try:
                request = self.requests.get(timeout=2)
            except queue.Empty:
                request = None
            if request is None and not automatic:
                continue
            try:
                if request is not None:
                    kind, value = request
                else:
                    current = active_network()
                    if current != previous:
                        previous = current
                        continue
                    kind, value = current
                data = render(kind, value, self.clips)
                self._put_ready((data, request is None))
                if request is None:
                    automatic = False
            except (OSError, RuntimeError, ValueError, KeyError, EOFError, wave.Error,
                    subprocess.SubprocessError) as error:
                if request is None:
                    previous = None
                else:
                    print("Network speech could not be prepared: {}".format(error),
                          file=sys.stderr, flush=True)

    def _listen(self):
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        identity = None
        try:
            try:
                current = self.socket_path.lstat()
            except FileNotFoundError:
                pass
            else:
                if not stat.S_ISSOCK(current.st_mode) or current.st_uid != os.geteuid():
                    raise RuntimeError("network speech socket path is invalid")
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                    try:
                        probe.connect(str(self.socket_path))
                    except (ConnectionRefusedError, FileNotFoundError):
                        self.socket_path.unlink(missing_ok=True)
                    else:
                        raise RuntimeError("network speech service is already running")
            listener.bind(str(self.socket_path))
            os.chmod(str(self.socket_path), 0o660)
            current = self.socket_path.lstat()
            identity = (current.st_dev, current.st_ino)
            listener.listen(MAX_PENDING)
            listener.settimeout(0.5)
            while not self.stopping.is_set():
                try:
                    connection, _ = listener.accept()
                except socket.timeout:
                    continue
                with connection:
                    try:
                        connection.settimeout(1)
                        data = bytearray()
                        while b"\n" not in data and len(data) < 32:
                            chunk = connection.recv(32 - len(data))
                            if not chunk:
                                break
                            data.extend(chunk)
                        command = data.decode("ascii").strip()
                        if command not in ("hotspot", "ip"):
                            response = b"invalid\n"
                        else:
                            try:
                                value = hotspot_name() if command == "hotspot" else address()
                                tokens(command, value)
                                self.requests.put_nowait((command, value))
                                response = b"queued\n"
                            except queue.Full:
                                response = b"full\n"
                            except (OSError, RuntimeError, ValueError, KeyError,
                                    subprocess.SubprocessError):
                                response = b"unavailable\n"
                        connection.sendall(response)
                    except (OSError, UnicodeError):
                        pass
        except (OSError, RuntimeError) as error:
            print("Network speech control unavailable: {}".format(error), file=sys.stderr, flush=True)
        finally:
            listener.close()
            if identity is not None:
                try:
                    current = self.socket_path.lstat()
                    if (current.st_dev, current.st_ino) == identity:
                        self.socket_path.unlink()
                except FileNotFoundError:
                    pass
