#!/usr/bin/env python3
"""Manage Wi-Fi recovery and the phone provisioning protocol."""

import argparse
import json
import logging
import os
from pathlib import Path
import queue
import socket
import socketserver
import tempfile
import threading
import time

from wifi_network import CONNECT_TIMEOUT, DEVICE_DISCONNECTED, NetworkError, NetworkManager


LOGGER = logging.getLogger("wifi-manager")
STATE_PATH = Path("/var/lib/wifi-manager/state.json")
PASSWORD_PATH = Path("/etc/wifi-manager/hotspot.psk")
DISCOVERY_REQUEST = b"LOBOT_NET_DISCOVER"
RESET_REQUEST = b"resetwifi"
DISCOVERY_TYPE = "WonderDuck"
HOTSPOT_PREFIX = "WD-"
STATION_RETRY_INTERVAL = 5
HOTSPOT_CHANNELS = (("a", 36), ("bg", 6))


def device_id(ethernet=Path("/sys/class/net/eth0/address"),
              serial=Path("/proc/device-tree/serial-number")):
    """Match the reference device's hotspot and discovery identifier."""
    if ethernet.exists():
        value = ethernet.read_text().strip().replace(":", "").upper()[-8:]
    else:
        value = serial.read_text()[-10:-1][:8]
    if not value:
        raise ValueError("device identifier is empty")
    return value


class StateStore:
    def __init__(self, path):
        self.path = Path(path)

    def load(self):
        try:
            with self.path.open(encoding="utf-8") as source:
                state = json.load(source)
        except FileNotFoundError:
            return None
        if (not isinstance(state, dict) or state.get("mode") not in ("ap", "station")
                or (state["mode"] == "station" and not isinstance(state.get("uuid"), str))
                or (state["mode"] == "station" and not state["uuid"])):
            raise ValueError("invalid Wi-Fi manager state")
        return state

    def save(self, state):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                         prefix=".state-", delete=False) as target:
            json.dump(state, target, separators=(",", ":"))
            target.write("\n")
            target.flush()
            os.fsync(target.fileno())
            temporary = target.name
        os.replace(temporary, self.path)


class Controller:
    def __init__(self, network, store, hotspot_name, hotspot_password, clock=time.monotonic,
                 *, hotspot_channels=HOTSPOT_CHANNELS):
        self.network = network
        self.store = store
        self.hotspot_name = hotspot_name
        self.hotspot_password = hotspot_password
        self.hotspot_channels = hotspot_channels
        self.clock = clock
        self.state = store.load()
        self.actions = queue.Queue()
        self.disconnected_since = clock()
        self.station_disconnected_since = None
        self.next_station_attempt = 0
        self.recovery_hotspot = False
        self.last_error = None

    def connect(self, ssid, password):
        self.actions.put(("connect", ssid, password))

    def reset(self):
        self.actions.put(("reset",))

    def _set_state(self, state):
        self.store.save(state)
        self.state = state
        self.disconnected_since = self.clock()
        self.station_disconnected_since = None
        self.next_station_attempt = 0
        self.recovery_hotspot = False

    def _hotspot(self):
        for band, channel in self.hotspot_channels:
            try:
                self.network.activate_hotspot(self.hotspot_name, self.hotspot_password,
                                              band, channel)
                LOGGER.info("hotspot started on band %s channel %s", band, channel)
                return
            except NetworkError as error:
                LOGGER.warning("hotspot attempt failed: %s", error)
        LOGGER.error("hotspot could not be started; will retry")

    def _switch_to_hotspot(self):
        self._set_state({"mode": "ap"})
        self._hotspot()

    def _recover_hotspot(self):
        if self.state is not None and self.state["mode"] == "station":
            LOGGER.info("starting recovery hotspot while keeping saved station profile")
            self.recovery_hotspot = True
            self._hotspot()
        else:
            self._switch_to_hotspot()

    def process_action(self, action):
        if action[0] == "reset":
            self._switch_to_hotspot()
        elif action[0] == "connect":
            _, ssid, password = action
            try:
                uuid = self.network.connect(ssid, password)
            except NetworkError as error:
                LOGGER.warning("phone provisioning failed: %s", error)
                self._recover_hotspot()
            else:
                self._set_state({"mode": "station", "uuid": uuid})
                LOGGER.info("phone provisioning completed")
        else:
            raise ValueError("unknown Wi-Fi manager action")

    def maintain(self):
        try:
            current = self.network.snapshot()
        except NetworkError as error:
            message = str(error)
            if message != self.last_error:
                LOGGER.warning("cannot read Wi-Fi state: %s", message)
                self.last_error = message
            return
        self.last_error = None

        if self.state is None:
            if current.mode == "station":
                self._set_state({"mode": "station", "uuid": current.uuid})
                LOGGER.info("using the current Wi-Fi connection")
            elif current.mode == "ap" and current.name == self.hotspot_name:
                self._set_state({"mode": "ap"})
            elif self.clock() - self.disconnected_since >= CONNECT_TIMEOUT:
                self._switch_to_hotspot()
            return

        if self.state["mode"] == "ap":
            if current.mode != "ap" or current.name != self.hotspot_name:
                self._hotspot()
            return

        # NetworkManager can activate another usable profile while the saved
        # profile is still waiting for the device or supplicant to initialize.
        if current.mode == "station":
            self.station_disconnected_since = None
            self.next_station_attempt = 0
            self.recovery_hotspot = False
            return
        if self.recovery_hotspot:
            if current.mode != "ap" or current.name != self.hotspot_name:
                self._hotspot()
            return
        if current.device_state < DEVICE_DISCONNECTED:
            # The device may take time to become available during boot.
            self.station_disconnected_since = None
            self.next_station_attempt = 0
            return
        now = self.clock()
        if self.station_disconnected_since is None:
            self.station_disconnected_since = now
        if now - self.station_disconnected_since >= CONNECT_TIMEOUT:
            self._recover_hotspot()
            return
        if current.can_activate and now >= self.next_station_attempt:
            self.next_station_attempt = now + STATION_RETRY_INTERVAL
            try:
                self.network.activate_saved(self.state["uuid"])
            except NetworkError as error:
                LOGGER.warning("saved Wi-Fi unavailable: %s", error)

    def run(self):
        while True:
            try:
                action = self.actions.get(timeout=1)
            except queue.Empty:
                self.maintain()
            else:
                self.process_action(action)


class ThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class DiscoveryHandler(socketserver.BaseRequestHandler):
    def handle(self):
        data, sock = self.request
        if data == DISCOVERY_REQUEST:
            sock.sendto(self.server.response, self.client_address)


class ProvisionHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2)
        data = bytearray()
        while True:
            try:
                chunk = self.request.recv(1024)
            except socket.timeout:
                LOGGER.warning("incomplete provisioning request")
                return
            if not chunk:
                LOGGER.warning("incomplete provisioning request")
                return
            data.extend(chunk)
            try:
                request = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(request, dict) or not isinstance(request.get("setwifi"), dict):
                LOGGER.warning("invalid provisioning request")
                return
            settings = request["setwifi"]
            if not isinstance(settings.get("ssid"), str) or not settings["ssid"] \
                    or not isinstance(settings.get("passwd"), str):
                LOGGER.warning("invalid provisioning fields")
                return
            self.server.controller.connect(settings["ssid"], settings["passwd"])
            return


class ResetHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2)
        data = bytearray()
        while len(data) < len(RESET_REQUEST):
            try:
                chunk = self.request.recv(1024)
            except socket.timeout:
                LOGGER.warning("incomplete reset request")
                return
            if not chunk:
                return
            data.extend(chunk)
        if data == RESET_REQUEST:
            self.server.controller.reset()


def discovery_server(identity, host):
    discovery = socketserver.UDPServer((host, 9027), DiscoveryHandler)
    discovery.response = (DISCOVERY_TYPE + ":" + identity + "\n").encode("utf-8")
    return discovery


def start_servers(controller, identity, host="0.0.0.0"):
    discovery = discovery_server(identity, host)
    provisioning = ThreadingTCPServer((host, 9026), ProvisionHandler)
    provisioning.controller = controller
    reset = ThreadingTCPServer((host, 9028), ResetHandler)
    reset.controller = controller
    servers = (discovery, provisioning, reset)
    for server in servers:
        threading.Thread(target=server.serve_forever, daemon=True).start()
    return servers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--discovery-only", action="store_true",
                        help="respond to device discovery while another service manages Wi-Fi")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    identity = device_id()
    if args.discovery_only:
        with discovery_server(identity, "0.0.0.0") as server:
            LOGGER.info("phone discovery endpoint started")
            server.serve_forever()
        return
    password = PASSWORD_PATH.read_text(encoding="utf-8").rstrip("\r\n")
    channels = HOTSPOT_CHANNELS[::-1] if os.environ.get("WONDERDUCK_HOTSPOT_2GHZ_FIRST") == "1" else HOTSPOT_CHANNELS
    controller = Controller(NetworkManager(), StateStore(STATE_PATH), HOTSPOT_PREFIX + identity,
                            password, hotspot_channels=channels)
    start_servers(controller, identity)
    LOGGER.info("phone provisioning endpoints started")
    controller.run()


if __name__ == "__main__":
    main()
