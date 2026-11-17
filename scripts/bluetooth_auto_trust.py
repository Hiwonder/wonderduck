#!/usr/bin/python3
"""Accept headless Bluetooth pairing and trust devices when they connect."""

import logging
import sys
from pathlib import Path
import configparser
import subprocess

import dbus
import dbus.service
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
from bluetooth_sixaxis import NAMES, register_pending


BLUEZ = "org.bluez"
ADAPTER = "org.bluez.Adapter1"
DEVICE = "org.bluez.Device1"
PROPERTIES = "org.freedesktop.DBus.Properties"
OBJECT_MANAGER = "org.freedesktop.DBus.ObjectManager"
AGENT_MANAGER = "org.bluez.AgentManager1"
AGENT_PATH = "/org/robot/bluetooth_agent"


class PairingRejected(dbus.DBusException):
    _dbus_error_name = "org.bluez.Error.Rejected"


class PairingAgent(dbus.service.Object):
    def __init__(self, bus, trust):
        super().__init__(bus, AGENT_PATH)
        self.trust = trust

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Release(self):
        logging.info("Bluetooth pairing agent released")

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        self.trust(device)

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        self.trust(device)

    @dbus.service.method("org.bluez.Agent1", in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        self.trust(device)

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        raise PairingRejected("This device requires a PIN")

    @dbus.service.method("org.bluez.Agent1", in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        raise PairingRejected("This device requires a passkey")

    @dbus.service.method("org.bluez.Agent1", in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pin):
        logging.info("A Bluetooth device requested PIN display")

    @dbus.service.method("org.bluez.Agent1", in_signature="ouq", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        logging.info("A Bluetooth device requested passkey display")

    @dbus.service.method("org.bluez.Agent1", in_signature="", out_signature="")
    def Cancel(self):
        logging.info("Bluetooth pairing request cancelled")


class AutoTrust:
    def __init__(self, bus):
        self.bus = bus
        self.agent = PairingAgent(bus, self.trust_device)
        self.registered = False
        self.attached = False
        bus.add_signal_receiver(self.interfaces_added, signal_name="InterfacesAdded",
                                dbus_interface=OBJECT_MANAGER, bus_name=BLUEZ)
        bus.add_signal_receiver(self.properties_changed, signal_name="PropertiesChanged",
                                dbus_interface=PROPERTIES, bus_name=BLUEZ, path_keyword="path")
        bus.add_signal_receiver(self.owner_changed, signal_name="NameOwnerChanged",
                                dbus_interface="org.freedesktop.DBus", arg0=BLUEZ)

    def properties(self, path):
        return dbus.Interface(self.bus.get_object(BLUEZ, path), PROPERTIES)

    def trust_device(self, path):
        try:
            properties = self.properties(path)
            if not properties.Get(DEVICE, "Trusted"):
                properties.Set(DEVICE, "Trusted", dbus.Boolean(True))
                logging.info("Bluetooth device trusted")
        except dbus.DBusException as error:
            logging.warning("Cannot trust Bluetooth device: %s", error)

    def prepare_adapter(self, path):
        try:
            properties = self.properties(path)
            if not properties.Get(ADAPTER, "Powered"):
                return
            for name, desired in (("PairableTimeout", dbus.UInt32(0)),
                                  ("DiscoverableTimeout", dbus.UInt32(0)),
                                  ("Pairable", dbus.Boolean(True)),
                                  ("Discoverable", dbus.Boolean(True))):
                if properties.Get(ADAPTER, name) != desired:
                    properties.Set(ADAPTER, name, desired)
        except dbus.DBusException as error:
            logging.warning("Cannot configure Bluetooth adapter: %s", error)

    def interfaces_added(self, path, interfaces):
        if ADAPTER in interfaces:
            self.prepare_adapter(path)
            if interfaces[ADAPTER].get("Powered"):
                self.schedule_controller_registration()
        device = interfaces.get(DEVICE)
        if device and str(device.get("Name", "")) in NAMES:
            self.schedule_controller_registration()
        if device and (device.get("Connected") or device.get("Paired")):
            self.trust_device(path)

    def properties_changed(self, interface, changed, invalidated, path=None):
        if interface == ADAPTER and any(name in changed for name in
                                        ("Powered", "Pairable", "Discoverable")):
            self.prepare_adapter(path)
            if changed.get("Powered"):
                self.schedule_controller_registration()
        elif interface == DEVICE:
            if str(changed.get("Name", "")) in NAMES:
                self.schedule_controller_registration()
            if changed.get("Connected") or changed.get("Paired"):
                self.trust_device(path)
            elif "Trusted" in changed and not changed["Trusted"]:
                try:
                    properties = self.properties(path)
                    if properties.Get(DEVICE, "Connected") or properties.Get(DEVICE, "Paired"):
                        self.trust_device(path)
                except dbus.DBusException as error:
                    logging.warning("Cannot inspect Bluetooth device: %s", error)

    def owner_changed(self, name, old_owner, new_owner):
        self.registered = False
        self.attached = False
        if new_owner:
            GLib.timeout_add_seconds(1, self.attach)

    def attach(self):
        if self.attached:
            return False
        try:
            manager = dbus.Interface(self.bus.get_object(BLUEZ, "/org/bluez"), AGENT_MANAGER)
            agent_path = dbus.ObjectPath(AGENT_PATH)
            if not self.registered:
                manager.RegisterAgent(agent_path, "NoInputNoOutput")
                self.registered = True
            manager.RequestDefaultAgent(agent_path)
            objects = dbus.Interface(self.bus.get_object(BLUEZ, "/"), OBJECT_MANAGER).GetManagedObjects()
            for path, interfaces in objects.items():
                self.interfaces_added(path, interfaces)
        except dbus.DBusException as error:
            logging.warning("Bluetooth agent unavailable, retrying: %s", error)
            return True
        self.attached = True
        logging.info("Bluetooth automatic trust is ready")
        return False

    def register_controllers(self):
        try:
            objects = dbus.Interface(self.bus.get_object(BLUEZ, "/"), OBJECT_MANAGER).GetManagedObjects()
            adapters = {str(path): str(interfaces[ADAPTER]["Address"]) for path, interfaces in objects.items()
                        if ADAPTER in interfaces and interfaces[ADAPTER].get("Powered")}
            connected = [(adapters.get(str(device.get("Adapter")), ""), str(device.get("Address", "")))
                         for interfaces in objects.values() for device in [interfaces.get(DEVICE, {})]
                         if device.get("Connected")]
            discovered = [(adapters.get(str(device.get("Adapter")), ""), str(device.get("Address", "")),
                           str(device.get("Name", "")))
                          for interfaces in objects.values() for device in [interfaces.get(DEVICE, {})]
                          if str(device.get("Name", "")) in NAMES]
            register_pending(Path("/var/lib/bluetooth"), adapters.values(), connected, discovered)
        except (dbus.DBusException, OSError, ValueError, configparser.Error, subprocess.SubprocessError) as error:
            logging.warning("Cannot complete PS3 controller registration: %s", error)
        return True

    def schedule_controller_registration(self):
        if not getattr(self, "registration_scheduled", False):
            self.registration_scheduled = True
            # Allow a new connection time to expose its input device before
            # deciding whether Bluetooth must be restarted for registration.
            GLib.timeout_add_seconds(3, self.register_controller_once)

    def register_controller_once(self):
        self.registration_scheduled = False
        self.register_controllers()
        return False


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s", stream=sys.stderr)
    DBusGMainLoop(set_as_default=True)
    service = AutoTrust(dbus.SystemBus())
    GLib.timeout_add_seconds(1, service.attach)
    GLib.timeout_add_seconds(10, service.register_controllers)
    GLib.MainLoop().run()


if __name__ == "__main__":
    main()
