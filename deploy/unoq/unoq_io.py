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
HX_READ = 0x02
HX_SYNC_READ = 0x82
HX_SYNC_WRITE = 0x83
UNOQ_FEEDBACK_READ = 0x93
UNOQ_FEEDBACK_METADATA_ID = 0xfd
UNOQ_FEEDBACK_TIMING_ID = 0xfc
UNOQ_FEEDBACK_IDS = bytes((20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14, 200))
UNOQ_FEEDBACK_LAYOUTS = {(56, 12): ("feedback", 2), (40, 28): ("feedback_extended", 3)}
STATUS_READ = 0x92
TCGETS2 = 0x802C542A
TIOCGPTPEER = 0x5441
TERMIOS2 = struct.Struct("=IIIIB19BII")
RPC_LOCK_DIRECTORY = Path("/run/lock/wonderduck-unoq")


class Router:
    def __init__(self, path=None, timeout=.1):
        endpoint = path or os.environ["WONDERDUCK_UNOQ_ROUTER"]
        self.endpoint = endpoint
        self.prioritize_hx = (
            Path(endpoint).resolve() == Path("/run/arduino-router.sock").resolve()
            and RPC_LOCK_DIRECTORY.is_dir()
        )
        self.socket = socket.socket(socket.AF_UNIX)
        try:
            self.socket.settimeout(timeout)
            self.socket.connect(endpoint)
        except BaseException:
            self.socket.close()
            raise
        self.responses = msgpack.Unpacker(raw=False, max_buffer_size=65536)
        self.ids = itertools.count(1)

    def call(self, method, *args, timed=False):
        if not self.prioritize_hx:
            return self._exchange(method, args, timed)

        with (RPC_LOCK_DIRECTORY / "admission").open("rb") as admission, \
             (RPC_LOCK_DIRECTORY / "wire").open("rb") as wire:
            if method in ("servo", "feedback", "feedback_extended"):
                # Keep the admission lock while HX waits for an in-flight
                # transaction. No new sensor exchange can overtake HX.
                fcntl.flock(admission, fcntl.LOCK_EX)
                fcntl.flock(wire, fcntl.LOCK_EX)
            else:
                while True:
                    fcntl.flock(admission, fcntl.LOCK_EX)
                    try:
                        fcntl.flock(wire, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        break
                    except BlockingIOError:
                        pass
                    finally:
                        fcntl.flock(admission, fcntl.LOCK_UN)
                    time.sleep(.001)
            return self._exchange(method, args, timed)

    def _exchange(self, method, args, timed):
        request_id = next(self.ids)
        request = msgpack.packb([0, request_id, "wonderduck/" + method, args], use_bin_type=True)
        exchange_start_ns = time.monotonic_ns() if timed else None
        self.socket.sendall(request)
        while True:
            for response in self.responses:
                kind, returned_id, error, value = response
                if kind != 1 or request_id != returned_id:
                    raise OSError("unexpected Arduino Router response")
                if error:
                    raise OSError(f"Arduino Router {method}: {error}")
                if timed:
                    return value, (exchange_start_ns, time.monotonic_ns())
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
        if os.environ.get("WONDERDUCK_UNOQ_HEAD_IMU_ENABLED", "true") != "true":
            raise OSError("UNO Q head IMU polling is disabled")
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
        if os.environ.get("WONDERDUCK_UNOQ_TOF_ENABLED", "true") != "true":
            raise OSError("UNO Q ToF polling is disabled")
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


def status_read_reply(router, frame):
    # The RPC response marks the end of the STM32's receive window. Report
    # an empty result explicitly so a disconnected joint cannot stall the scan.
    body = frame[2:4] + bytes((HX_READ,)) + frame[5:7]
    reply = router.call("servo", b"\xff\xff" + body + bytes((~sum(body) & 0xff,)))
    if reply == b"":
        body = bytes((frame[2], 2, 0))
        return b"\xff\xff" + body + bytes((~sum(body) & 0xff,))
    if isinstance(reply, bytes) and len(reply) == 6 and reply[3:5] == b"\x02\x00":
        raise OSError("empty success frame from the HX bus")
    return reply


def feedback_reply(sequence, age_us, reply, exchange_times=None):
    if not isinstance(reply, bytes) or not reply or not sequence or age_us > 40_000:
        return b""
    payload = struct.pack("<II", sequence, age_us)
    body = bytes((UNOQ_FEEDBACK_METADATA_ID, len(payload) + 2, 0)) + payload
    metadata = b"\xff\xff" + body + bytes((~sum(body) & 0xff,))
    if exchange_times is not None:
        timing = bytes((UNOQ_FEEDBACK_TIMING_ID, 18, 0)) + struct.pack("<QQ", *exchange_times)
        metadata += b"\xff\xff" + timing + bytes((~sum(timing) & 0xff,))
    return metadata + reply


def feedback_error_reply():
    body = bytes((UNOQ_FEEDBACK_METADATA_ID, 10, 0)) + bytes(8)
    return b"\xff\xff" + body + bytes((~sum(body) & 0xff,))


def valid_feedback_frames(data, payload_length):
    frame_length = payload_length + 6
    if len(data) != len(UNOQ_FEEDBACK_IDS) * frame_length:
        return False
    seen = set()
    for offset in range(0, len(data), frame_length):
        frame = data[offset:offset + frame_length]
        if (frame[:2] != b"\xff\xff" or frame[3] != payload_length + 2 or frame[2] not in UNOQ_FEEDBACK_IDS
                or frame[2] in seen or sum(frame[2:]) & 0xff != 0xff):
            return False
        seen.add(frame[2])
    return len(seen) == len(UNOQ_FEEDBACK_IDS)


def ordinary_sync_read(frame):
    request = bytearray(frame)
    request[4] = HX_SYNC_READ
    request[-1] = ~sum(request[2:-1]) & 0xff
    return bytes(request)


class FeedbackSession:
    def __init__(self, router):
        self.router = router
        self.version = None
        self.active = False
        self.sample_sequence = 0
        self.sequence = 0
        self.last_failure_log = 0

    def failure(self, reason):
        now = time.monotonic()
        if now - self.last_failure_log >= 1:
            print(f"STM32 feedback unavailable: {reason}", flush=True)
            self.last_failure_log = now
        return feedback_error_reply()

    def stop(self):
        if self.active:
            self.router.call("feedback_stop")
            self.active = False

    def read(self, frame):
        if (len(frame) != 24 or frame[2] != 0xfe or frame[7:-1] != UNOQ_FEEDBACK_IDS
                or tuple(frame[5:7]) not in UNOQ_FEEDBACK_LAYOUTS):
            raise OSError("invalid UNO Q cached feedback request")
        method, required_version = UNOQ_FEEDBACK_LAYOUTS[tuple(frame[5:7])]
        payload_length = frame[6]
        if self.version is None:
            try:
                self.version = self.router.call("feedback_version")
            except OSError as error:
                if "not available" not in str(error):
                    raise
                self.version = 0
        if self.version < required_version:
            self.stop()
            result = self.router.call("servo", ordinary_sync_read(frame))
            if not isinstance(result, bytes) or not valid_feedback_frames(result, payload_length):
                return self.failure("invalid synchronous feedback frames")
            return self.reply(0, result)

        self.active = True
        started = time.monotonic_ns()
        result, exchange_times = self.router.call(method, self.sample_sequence, timed=True)
        elapsed_us = (time.monotonic_ns() - started + 999) // 1000
        if not isinstance(result, bytes) or len(result) < 8:
            return self.failure("no new sample before deadline")
        sequence, mcu_age_us = struct.unpack_from("<II", result)
        advance = (sequence - self.sample_sequence) & 0xffffffff
        if sequence == 0 or advance == 0 or (self.sample_sequence != 0 and advance >= 0x80000000):
            return self.failure(f"invalid sample sequence={sequence}")
        # Bound the age before forwarding to the PTY. The RPC timestamps let
        # robotd account for when the MCU calculated its age within the exchange.
        age_us = mcu_age_us + elapsed_us
        data = result[8:]
        if not valid_feedback_frames(data, payload_length):
            return self.failure(f"invalid sample sequence={sequence} bytes={len(data)}")
        if age_us > 40_000:
            return self.failure(f"stale sample sequence={sequence} age_us={age_us}")
        self.sample_sequence = sequence
        return self.reply(mcu_age_us, data, advance, exchange_times)

    def reply(self, age_us, data, advance=1, exchange_times=None):
        # Preserve gaps in MCU sample numbers across synchronous fallback reads.
        self.sequence = (self.sequence + advance) & 0xffffffff or 1
        return feedback_reply(self.sequence, age_us, data, exchange_times)


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
    feedback = FeedbackSession(router)
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
                feedback.stop()
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
            if frame[4] not in (UNOQ_FEEDBACK_READ, HX_SYNC_WRITE):
                feedback.stop()
            if len(frame) == 8 and frame[2] < 254 and frame[4] == STATUS_READ:
                reply = status_read_reply(router, frame)
            elif frame[4] == UNOQ_FEEDBACK_READ:
                reply = feedback.read(frame)
            else:
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


def connect_bridge(timeout):
    deadline = time.monotonic() + timeout
    while True:
        router = None
        try:
            router = Router()
            version = router.call("version")
        except OSError:
            if router is not None:
                router.close()
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.25)
            continue
        if version != 1:
            router.close()
            raise OSError("unsupported WonderDuck STM32 bridge version")
        return router


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=Path, help="PTY link for the HX motor and AT32 bus")
    parser.add_argument("--owner", help="User allowed to open the PTY")
    options = parser.parse_args()
    # The router socket can exist before the STM32 registers its RPC methods.
    with connect_bridge(25 if options.port else 0) as router:
        if options.port:
            serve_bus(router, options.port, pwd.getpwnam(options.owner) if options.owner else None)
        else:
            print("WonderDuck STM32 bridge protocol 1")


if __name__ == "__main__":
    main()
