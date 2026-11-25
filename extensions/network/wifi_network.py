"""NetworkManager operations for the Wi-Fi provisioning service."""

import time
from dataclasses import dataclass


BUS_NAME = "org.freedesktop.NetworkManager"
MANAGER_PATH = "/org/freedesktop/NetworkManager"
SETTINGS_PATH = MANAGER_PATH + "/Settings"
PROPERTIES = "org.freedesktop.DBus.Properties"
DEVICE = BUS_NAME + ".Device"
ACTIVE = BUS_NAME + ".Connection.Active"
CONNECTION = BUS_NAME + ".Settings.Connection"
ACTIVATED = 2
DEACTIVATING = 3
DEVICE_ACTIVATED = 100
DEVICE_DISCONNECTED = 30
DEVICE_FAILED = 120
CONNECT_TIMEOUT = 30


class NetworkError(RuntimeError):
    """A NetworkManager operation failed or did not finish."""


@dataclass(frozen=True)
class Snapshot:
    device_state: int
    mode: str = ""
    uuid: str = ""
    name: str = ""

    @property
    def can_activate(self):
        return self.mode == "ap" or self.device_state in (DEVICE_DISCONNECTED, DEVICE_FAILED)


class NetworkManager:
    """Use NetworkManager's system-bus interface; it owns network credentials."""

    def __init__(self, interface="wlan0"):
        import dbus

        self.dbus = dbus
        self.interface = interface
        self.bus = dbus.SystemBus()
        self.manager = self._interface(MANAGER_PATH, BUS_NAME)
        self.settings = self._interface(SETTINGS_PATH, BUS_NAME + ".Settings")

    def _interface(self, path, interface):
        return self.dbus.Interface(self.bus.get_object(BUS_NAME, path), interface)

    def _property(self, path, interface, name):
        return self._interface(path, PROPERTIES).Get(interface, name)

    def _device(self):
        return self.manager.GetDeviceByIpIface(self.interface)

    def snapshot(self):
        try:
            device = self._device()
            state = int(self._property(device, DEVICE, "State"))
            active = self._property(device, DEVICE, "ActiveConnection")
            if state != DEVICE_ACTIVATED or str(active) == "/":
                return Snapshot(state)
            profile = self._property(active, ACTIVE, "Connection")
            settings = self._interface(profile, CONNECTION).GetSettings()
            mode = str(settings["802-11-wireless"].get("mode", "infrastructure"))
            if mode not in ("infrastructure", "ap"):
                raise NetworkError("unsupported active Wi-Fi mode: " + mode)
            return Snapshot(state, "ap" if mode == "ap" else "station",
                            str(self._property(active, ACTIVE, "Uuid")),
                            str(self._property(active, ACTIVE, "Id")))
        except self.dbus.DBusException as error:
            raise NetworkError("NetworkManager status failed: " + str(error)) from error

    def _encode(self, sections):
        encoded = {}
        for section, values in sections.items():
            converted = {}
            for key, value in values.items():
                if key == "ssid":
                    value = self.dbus.ByteArray(value.encode("utf-8"))
                elif key == "address-data":
                    value = self.dbus.Array([
                        self.dbus.Dictionary({"address": address,
                                              "prefix": self.dbus.UInt32(prefix)},
                                             signature="sv")
                        for address, prefix in value
                    ], signature="a{sv}")
                elif key == "channel":
                    value = self.dbus.UInt32(value)
                elif isinstance(value, bool):
                    value = self.dbus.Boolean(value)
                converted[key] = value
            encoded[section] = self.dbus.Dictionary(converted, signature="sv")
        return self.dbus.Dictionary(encoded, signature="sa{sv}")

    def _wait_active(self, active):
        deadline = time.monotonic() + CONNECT_TIMEOUT
        while time.monotonic() < deadline:
            state = int(self._property(active, ACTIVE, "State"))
            if state == ACTIVATED:
                return
            if state >= DEACTIVATING:
                raise NetworkError("NetworkManager rejected the connection")
            time.sleep(1)
        raise NetworkError("NetworkManager connection timed out")

    def _add_and_activate(self, sections):
        options = self.dbus.Dictionary({"persist": "volatile"}, signature="sv")
        profile, active, _ = self.manager.AddAndActivateConnection2(
            self._encode(sections), self._device(), self.dbus.ObjectPath("/"), options)
        self._wait_active(active)
        return profile, active

    def activate_hotspot(self, name, password, band, channel):
        sections = {
            "connection": {"id": name, "type": "802-11-wireless",
                           "interface-name": self.interface, "autoconnect": False},
            "802-11-wireless": {"ssid": name, "mode": "ap", "band": band,
                                "channel": channel},
            "ipv4": {"method": "shared", "address-data": [("192.168.149.1", 24)],
                     "never-default": True},
            "ipv6": {"method": "disabled"},
        }
        if password:
            sections["802-11-wireless-security"] = {"key-mgmt": "wpa-psk", "psk": password}
        try:
            self._add_and_activate(sections)
        except (self.dbus.DBusException, NetworkError) as error:
            message = str(error).replace(password, "[redacted]") if password else str(error)
            raise NetworkError("hotspot activation failed: " + message) from None

    def connect(self, ssid, password):
        sections = {
            "connection": {"id": ssid, "type": "802-11-wireless",
                           "interface-name": self.interface, "autoconnect": False},
            "802-11-wireless": {"ssid": ssid, "mode": "infrastructure"},
            "ipv4": {"method": "auto"},
            "ipv6": {"method": "auto"},
        }
        if password:
            sections["802-11-wireless-security"] = {"key-mgmt": "wpa-psk", "psk": password}
        try:
            profile, active = self._add_and_activate(sections)
            uuid = str(self._property(active, ACTIVE, "Uuid"))
            sections["connection"]["uuid"] = uuid
            sections["connection"]["autoconnect"] = True
            self._interface(profile, CONNECTION).Update2(
                self._encode(sections), self.dbus.UInt32(1),
                self.dbus.Dictionary({}, signature="sv"))
            return uuid
        except (self.dbus.DBusException, NetworkError) as error:
            message = str(error).replace(password, "[redacted]") if password else str(error)
            raise NetworkError("Wi-Fi connection failed: " + message) from None

    def activate_saved(self, uuid):
        try:
            profile = self.settings.GetConnectionByUuid(uuid)
            self.manager.ActivateConnection(profile, self._device(), self.dbus.ObjectPath("/"))
        except self.dbus.DBusException as error:
            raise NetworkError("saved Wi-Fi activation failed: " + str(error)) from error
