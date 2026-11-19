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
SETTINGS = ROOT.parent / ".config/wonderduck-studio/network-announcement.json"
INTERFACE = "wlan0"
MAX_PENDING = 8
MAX_REQUEST_BYTES = 512
SWITCH_TIMEOUT_SECONDS = 90
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


def active_network_state():
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
        return "hotspot", name, name
    if mode in ("infrastructure", "", "--"):
        name = _run("nmcli", "-e", "no", "-g", "802-11-wireless.ssid",
                    "connection", "show", "uuid", uuid)
        return "ip", address(), name
    raise RuntimeError("wireless mode is unavailable")


def active_network():
    kind, value, _ssid = active_network_state()
    return kind, value


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


class NetworkSwitch:
    """Wait for the requested network and a stable address or hotspot name."""

    def __init__(self, mode, ssid, generation, now):
        self.kind = "hotspot" if mode == "ap" else "ip"
        self.ssid = ssid
        self.generation = generation
        self.deadline = now + SWITCH_TIMEOUT_SECONDS
        self.previous = None

    def observe(self, state):
        kind, value, ssid = state
        if kind != self.kind or (self.kind == "ip" and self.ssid and ssid != self.ssid):
            self.previous = None
            return None
        if state == self.previous:
            return kind, value
        self.previous = state
        return None


class NetworkAnnouncements:
    def __init__(self, clips=CLIPS, marker=MARKER, socket_path=SOCKET, settings=SETTINGS):
        self.clips = clips
        self.marker = marker
        self.socket_path = socket_path
        self.settings = settings
        self.requests = queue.Queue(maxsize=MAX_PENDING)
        self.ready = queue.Queue(maxsize=MAX_PENDING)
        self.stopping = threading.Event()
        self.generation_lock = threading.Lock()
        self.generation = 0
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

    def automatic_enabled(self):
        """Read the owner's preference even when this service runs as root."""
        try:
            data = json.loads(self.settings.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or type(data.get("enabled")) is not bool:
                raise ValueError("invalid network announcement setting")
            return data["enabled"]
        except FileNotFoundError:
            return True
        except (OSError, ValueError, UnicodeError) as error:
            print("Cannot read network announcement setting: {}".format(error),
                  file=sys.stderr, flush=True)
            return False

    def current_generation(self):
        with self.generation_lock:
            return self.generation

    def queue_switch(self, mode, ssid=None):
        if mode not in ("ap", "station"):
            raise ValueError("invalid network mode")
        if mode == "station" and (not isinstance(ssid, str) or not ssid
                                  or len(ssid.encode("utf-8")) > 32):
            raise ValueError("invalid wireless network name")
        enabled = self.automatic_enabled()
        with self.generation_lock:
            self.generation += 1
            generation = self.generation
            if not enabled:
                return False
            self.requests.put_nowait(("watch", mode, ssid, generation))
        return True

    def _work(self):
        automatic = not self.marker.exists() and self.automatic_enabled()
        previous = None
        pending = None
        while not self.stopping.is_set():
            try:
                request = self.requests.get(timeout=2)
            except queue.Empty:
                request = None
            if request is not None and request[0] == "watch":
                _, mode, ssid, generation = request
                pending = NetworkSwitch(mode, ssid, generation, time.monotonic())
                automatic = False
                previous = None
                continue
            if request is None and not (automatic or pending):
                continue
            try:
                if request is not None:
                    kind, value = request
                    generation = None
                    is_automatic = False
                elif not self.automatic_enabled():
                    automatic = False
                    pending = None
                    continue
                elif pending is not None:
                    if (time.monotonic() >= pending.deadline
                            or pending.generation != self.current_generation()):
                        pending = None
                        continue
                    result = pending.observe(active_network_state())
                    if result is None:
                        continue
                    kind, value = result
                    generation = pending.generation
                    is_automatic = True
                else:
                    generation = self.current_generation()
                    current = active_network()
                    if generation != self.current_generation():
                        continue
                    if current != previous:
                        previous = current
                        continue
                    kind, value = current
                    is_automatic = True
                data = render(kind, value, self.clips)
                self._put_ready((data, is_automatic, generation))
                if is_automatic:
                    automatic = False
                    pending = None
            except (OSError, RuntimeError, ValueError, KeyError, EOFError, wave.Error,
                    subprocess.SubprocessError) as error:
                if request is None:
                    previous = None
                    if pending is not None:
                        pending.previous = None
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
                        while b"\n" not in data and len(data) < MAX_REQUEST_BYTES:
                            chunk = connection.recv(MAX_REQUEST_BYTES - len(data))
                            if not chunk:
                                break
                            data.extend(chunk)
                        command = data.decode("ascii").strip()
                        if command in ("hotspot", "ip"):
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
                        else:
                            try:
                                message = json.loads(command)
                                if not isinstance(message, dict) or set(message) - {"watch", "ssid"}:
                                    raise ValueError("invalid network speech request")
                                queued = self.queue_switch(message.get("watch"), message.get("ssid"))
                                response = b"queued\n" if queued else b"disabled\n"
                            except queue.Full:
                                response = b"full\n"
                            except (TypeError, ValueError):
                                response = b"invalid\n"
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
