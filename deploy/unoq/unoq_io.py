"""WonderDuck expansion-board transport over the Arduino Router Unix socket."""

import argparse
import errno
import fcntl
import itertools
import os
from pathlib import Path
import pty
import pwd
import select
import socket
import struct
import termios
import time
import tty

import msgpack

HX_BAUD = 1_000_000
TCGETS2 = 0x802C542A
TIOCGPTPEER = 0x5441
TERMIOS2 = struct.Struct("=IIIIB19BII")


class Router:
    def __init__(self, path=None, timeout=.1):
        self.socket = socket.socket(socket.AF_UNIX)
        try:
            self.socket.settimeout(timeout)
            self.socket.connect(path or os.environ["WONDERDUCK_UNOQ_ROUTER"])
        except BaseException:
            self.socket.close()
            raise
        self.responses = msgpack.Unpacker(raw=False, max_buffer_size=65536)
        self.ids = itertools.count(1)

    def call(self, method, *args):
        request_id = next(self.ids)
        self.socket.sendall(msgpack.packb([0, request_id, "wonderduck/" + method, args],
                                          use_bin_type=True))
        while True:
            for response in self.responses:
                kind, returned_id, error, value = response
                if kind != 1 or request_id != returned_id:
                    raise OSError("unexpected Arduino Router response")
                if error:
                    raise OSError(f"Arduino Router {method}: {error}")
                return value
            chunk = self.socket.recv(4096)
            if not chunk:
                raise OSError("Arduino Router disconnected")
            self.responses.feed(chunk)

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def read_i2c_reply(reply, expected):
    if not isinstance(reply, bytes) or len(reply) != expected + 1 or reply[0] != 0:
        raise OSError("WonderDuck expansion-board I2C device did not respond")
    return reply[1:]


class HeadImu:
    def __init__(self, address):
        self.address = address
        self.router = Router()

    def read(self, register, length=1):
        reply = self.router.call("head_imu", self.address, bytes((register,)), length)
        return read_i2c_reply(reply, length)

    def write(self, register, value):
        reply = self.router.call("head_imu", self.address, bytes((register, value)), 0)
        read_i2c_reply(reply, 0)

    def close(self):
        self.router.close()


class TofBus:
    """The CircuitPython I2C subset used by the existing VL53L0X driver."""

    def __init__(self):
        self.router = Router()
        self.locked = False

    def try_lock(self):
        if self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        self.locked = False

    def deinit(self):
        self.router.close()

    def transfer(self, address, request, length):
        if address != 0x29:
            raise ValueError("the expansion-board ToF port uses address 0x29")
        return read_i2c_reply(self.router.call("tof", bytes(request), length), length)

    def writeto(self, address, buffer, *, start=0, end=None):
        self.transfer(address, memoryview(buffer)[start:end], 0)

    def readfrom_into(self, address, buffer, *, start=0, end=None):
        destination = memoryview(buffer)[start:end]
        destination[:] = self.transfer(address, b"", len(destination))

    def writeto_then_readfrom(self, address, output, input_buffer, *,
                              out_start=0, out_end=None, in_start=0, in_end=None):
        destination = memoryview(input_buffer)[in_start:in_end]
        destination[:] = self.transfer(address, memoryview(output)[out_start:out_end],
                                       len(destination))


def servo_packets(buffer):
    while len(buffer) >= 4:
        start = buffer.find(b"\xff\xff")
        if start < 0:
            del buffer[:-1]
            break
        del buffer[:start]
        if len(buffer) < 4:
            break
        length = buffer[3] + 4
        if length < 6:
            del buffer[0]
            continue
        if len(buffer) < length:
            break
        frame = bytes(buffer[:length])
        if sum(frame[2:]) & 0xff != 0xff:
            del buffer[0]
            continue
        del buffer[:length]
        yield frame


def release_closed_client(master):
    # TIOCEXCL survives a slave close while the master exists. Linux allows
    # the bridge to reopen the peer as root and clear it once the client lock
    # has been released. A client acquiring the PTY now has to take that lock.
    peer = fcntl.ioctl(master, TIOCGPTPEER, os.O_RDWR | os.O_NOCTTY)
    try:
        try:
            fcntl.flock(peer, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        fcntl.ioctl(peer, termios.TIOCNXCL)
        termios.tcflush(peer, termios.TCIOFLUSH)
    finally:
        os.close(peer)


def serve_bus(router, port, owner=None):
    # Hold the master across maintenance handoffs. Close our own slave so
    # Studio's fuser check reports only real clients of the motor bus.
    lock_path = port.with_suffix(".lock")
    with lock_path.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        master, slave = pty.openpty()
        try:
            try:
                tty.setraw(slave)
                link = Path(os.ttyname(slave))
                if owner is not None:
                    os.fchown(slave, owner.pw_uid, owner.pw_gid)
                os.fchmod(slave, 0o600)
            finally:
                os.close(slave)
            port.symlink_to(link)
            try:
                forward_servo_packets(router, master)
            finally:
                port.unlink()
        finally:
            os.close(master)


def forward_servo_packets(router, master):
    pending = bytearray()
    serial_settings = bytearray(TERMIOS2.size)
    closed = False
    events = select.poll()
    events.register(master, 0)
    while True:
        try:
            data = os.read(master, 4096)
        except OSError as error:
            if error.errno != errno.EIO:
                raise
            if not closed:
                pending.clear()
                release_closed_client(master)
                closed = True
            time.sleep(0.02)
            continue
        closed = False
        pending.extend(data)
        for frame in servo_packets(pending):
            fcntl.ioctl(master, TCGETS2, serial_settings)
            if TERMIOS2.unpack(serial_settings)[-2:] != (HX_BAUD, HX_BAUD):
                continue
            reply = router.call("servo", frame)
            if not isinstance(reply, bytes):
                raise OSError("malformed UART reply from expansion board")
            if not reply or events.poll(0):
                continue
            try:
                if os.write(master, reply) != len(reply):
                    raise OSError("incomplete UART reply to the local client")
            except OSError as error:
                if error.errno != errno.EIO:
                    raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, help="PTY link for the HX motor and AT32 bus")
    parser.add_argument("--owner", help="User allowed to open the PTY")
    options = parser.parse_args()
    with Router() as router:
        if router.call("version") != 1:
            raise OSError("unsupported WonderDuck STM32 bridge version")
        if options.port:
            serve_bus(router, options.port, pwd.getpwnam(options.owner) if options.owner else None)
        else:
            print("WonderDuck STM32 bridge protocol 1")


if __name__ == "__main__":
    main()
