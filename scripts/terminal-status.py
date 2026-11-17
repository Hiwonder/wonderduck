#!/usr/bin/env python3
"""Print the device version, addresses and current battery at shell startup."""

import ast
import json
import math
from pathlib import Path
import socket
import subprocess
import sys


VERSION_FILE = Path('/etc/robot/device-version.json')
CONFIG_FILE = Path('/etc/robot/robotd.toml')
ROBOT_SOCKET = Path('/run/robotd.sock')
STATUS_JOURNAL = Path('/var/lib/robotd-status-read.pending')
SERVO_READER = Path(__file__).with_name('servo-live.py')
IMU_READER = Path(__file__).with_name('imu-live.py')


def versions(path):
    try:
        data = json.loads(path.read_text())
        hardware = data['hardware_version']
        image = data['image_version']
        if not all(isinstance(value, str) and value.strip() for value in (hardware, image)):
            raise ValueError('invalid device versions')
        return f'hardware {hardware} | image {image}'
    except (OSError, ValueError, KeyError, TypeError):
        return 'Unknown'


def addresses():
    try:
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True,
                                check=True, timeout=1)
        return ' '.join(result.stdout.split()) or 'Not connected'
    except (OSError, subprocess.SubprocessError):
        return 'Unknown'


def robot_battery(path):
    """Return whether robotd answered and its battery, without touching the bus."""
    connected = False
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(0.5)
            client.connect(str(path))
            connected = True
            client.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.health"}\n')
            with client.makefile('rb') as reader:
                response = json.loads(reader.readline(65536))
        if response.get('id') != 1:
            return True, None
        battery = response['result']['battery']
        volts = float(battery['volts'])
        if not math.isfinite(volts) or volts <= 0:
            return True, None
        return True, volts
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return connected, None


def robot_firmware(path):
    """Get the ID200 version from robotd without sharing its serial bus."""
    connected = False
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(0.5)
            client.connect(str(path))
            connected = True
            client.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.health"}\n')
            with client.makefile('rb') as reader:
                response = json.loads(reader.readline(65536))
        version = response['result'].get('imu_firmware_version')
        if response.get('id') == 1 and isinstance(version, str) and version:
            return True, version
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass
    return connected, None


def hx_port(path):
    """Read the HX bus settings used by robotd; do not guess another bus type."""
    try:
        section = ''
        settings = {}
        for raw_line in path.read_text().splitlines():
            line = raw_line.strip()
            if line.startswith('['):
                section = line.split('#', 1)[0].strip()
            elif section == '[bus]' and '=' in line:
                name, value = line.split('=', 1)
                if name.strip() in ('protocol', 'port'):
                    settings[name.strip()] = ast.literal_eval(value.strip())
        port = settings.get('port')
        if settings.get('protocol') == 'hx' and isinstance(port, str) and port.startswith('/dev/'):
            return port
    except (OSError, SyntaxError, ValueError):
        pass
    return None


def bus_battery(config_path, reader_path):
    """Use the existing read-only HX tool, which takes the serial lock."""
    if status_bus_locked():
        return None
    port = hx_port(config_path)
    if port is None:
        return None
    try:
        result = subprocess.run(
            [sys.executable, str(reader_path), '--port', port, '--ids', '20',
             '--mode', 'read', '--once', '--timeout-ms', '100'],
            capture_output=True, text=True, check=True, timeout=2,
        )
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 8 and fields[:2] == ['20', 'OK']:
                volts = float(fields[6])
                if math.isfinite(volts) and volts > 0:
                    return volts
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    return None


def status_bus_locked():
    """Never send an automatic request while a status reply is unconfirmed."""
    try:
        STATUS_JOURNAL.lstat()
    except FileNotFoundError:
        return False
    except OSError:
        return True
    return True


def battery():
    answered, reading = robot_battery(ROBOT_SOCKET)
    if answered:
        return reading
    # robotd can be starting before its socket is ready. Do not open its bus then.
    try:
        status = subprocess.run(['pgrep', '-x', 'robotd'], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=0.5).returncode
    except (OSError, subprocess.SubprocessError):
        return None
    return bus_battery(CONFIG_FILE, SERVO_READER) if status == 1 else None


def firmware():
    answered, version = robot_firmware(ROBOT_SOCKET)
    if answered:
        return version
    try:
        status = subprocess.run(['pgrep', '-x', 'robotd'], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=0.5).returncode
        port = hx_port(CONFIG_FILE)
        if status != 1 or port is None or status_bus_locked():
            return None
        result = subprocess.run([sys.executable, str(IMU_READER), '--port', port,
                                 '--firmware-info', '--timeout-ms', '200'],
                                capture_output=True, text=True, check=True, timeout=2)
        info = json.loads(result.stdout)
        version = info.get('firmware_version')
        return version if isinstance(version, str) and version else None
    except (OSError, ValueError, subprocess.SubprocessError, AttributeError):
        return None


def main():
    reading = battery()
    charge = f'{reading:.2f} V' if reading is not None else 'Unavailable'
    print(f'Version: {versions(VERSION_FILE)} | firmware {firmware() or "Unavailable"}')
    print(f'IP: {addresses()}')
    print(f'Battery: {charge}')


if __name__ == '__main__':
    main()
