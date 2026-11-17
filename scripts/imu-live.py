#!/usr/bin/env python3
"""Read ID200 raw six-axis measurements or inspect the fused policy input.

ID200 defaults to two read-only raw sensor queries; their samples may differ.
``--policy-view`` selects the fused output, and ``--all-devices`` also reads
the IMU and all 15 joints in one broadcast. ``--firmware-info`` reads build
identity once. No motor commands are sent.
"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import termios
import time
import importlib.util


PORT_DEFAULT = "/dev/ttyS3"
IMU_ID = 200
JOINT_IDS = (20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14)
ALL_DEVICE_IDS = (IMU_ID, *JOINT_IDS)
READ_ADDRESS = 56
READ_LENGTH = 12
READ_TIMEOUT_SECONDS = 0.040
FAULT_ADDRESS = 208
FAULT_LENGTH = 52
FAULT_TIMEOUT_SECONDS = 0.005
FIRMWARE_INFO_ADDRESS = 119
FIRMWARE_INFO_LENGTH = 64
GYRO_RAD_PER_LSB = 0.035 * math.pi / 180.0
# Facing the robot, the HX sensor is physically +X left, +Y up, +Z forward.
# This is the same sensor-to-trunk rotation as duck-control/src/hx.rs:
# trunk = [sensor_z, sensor_x, sensor_y].
HX_MOUNT = (0.5, 0.5, 0.5, 0.5)
GYRO_CALIBRATION_PATH = Path("/etc/robot/imu-gyro-fixed.json")
# Compensation bound, not a stationary test or sensor accuracy specification.
# Keep in sync with robotd/src/imu_calibration.rs.
GYRO_BIAS_MAX_NORM_RAD_S = struct.unpack("<f", struct.pack("<f", 0.30))[0]
_GAINS_SPEC = importlib.util.spec_from_file_location("imu_gyro_gains", Path(__file__).with_name("imu_gyro_gains.py"))
GAINS = importlib.util.module_from_spec(_GAINS_SPEC)
_GAINS_SPEC.loader.exec_module(GAINS)


def validate_gyro_calibration(report, port, imu_id=IMU_ID):
    """Validate the same saved calibration contract as robotd before using it."""
    if not isinstance(report, dict) or not isinstance(report.get("source"), dict):
        raise ValueError("陀螺仪校准文件缺少 source。")
    if (report.get("schema_version") != 1
            or report.get("frame") != "sensor"
            or report.get("mode") != "fixed"
            or report.get("units") != "rad/s"
            or report["source"].get("imu_id") != imu_id
            or report["source"].get("port") != port
            or not isinstance(report.get("created_utc"), str)
            or not report["created_utc"]):
        raise ValueError("陀螺仪校准的格式、坐标系、安装方向或数据来源与当前 HX 不一致。")
    for key in ("gyro_bias_rad_s", "verification_corrected_mean_rad_s"):
        vector = report.get(key)
        if (not isinstance(vector, list) or len(vector) != 3
                or any(type(value) not in (int, float) or not math.isfinite(value)
                       for value in vector)):
            raise ValueError(f"陀螺仪校准字段 {key} 必须包含三个有限数值。")
    bias = report["gyro_bias_rad_s"]
    if math.sqrt(sum(value * value for value in bias)) > GYRO_BIAS_MAX_NORM_RAD_S:
        raise ValueError(f"陀螺仪零偏模长超过可补偿上限 {GYRO_BIAS_MAX_NORM_RAD_S:.2f} rad/s。")
    if max(map(abs, report["verification_corrected_mean_rad_s"])) > 0.002:
        raise ValueError("陀螺仪独立验证残差超过允许范围。")
    GAINS.validate_configuration(report.get("gyro_gains"))
    return tuple(bias)


def load_gyro_calibration(port, path=GYRO_CALIBRATION_PATH, imu_id=IMU_ID):
    try:
        payload = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    return validate_gyro_calibration(json.loads(payload), port, imu_id)


class FrameParser:
    """Incrementally extracts valid HX status frames from a byte stream."""

    def __init__(self):
        self.buffer = bytearray()
        self.checksum_errors = 0
        self.invalid_lengths = 0

    def feed(self, data):
        self.buffer.extend(data)

    def pop(self):
        while True:
            if len(self.buffer) < 4:
                return None
            start = self.buffer.find(b"\xff\xff")
            if start < 0:
                if self.buffer[-1:] == b"\xff":
                    self.buffer[:] = b"\xff"
                else:
                    self.buffer.clear()
                return None
            if start:
                del self.buffer[:start]
            length = self.buffer[3]
            if length < 2:
                self.invalid_lengths += 1
                del self.buffer[0]
                continue
            total = length + 4
            if len(self.buffer) < total:
                return None
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            if sum(frame[2:]) & 0xFF != 0xFF:
                self.checksum_errors += 1
                continue
            return frame[2], frame[4], frame[5:-1]


def make_sync_read(ids):
    params = (READ_ADDRESS, READ_LENGTH, *ids)
    body = (0xFE, len(params) + 2, 0x82, *params)
    return bytes((0xFF, 0xFF, *body, (~sum(body)) & 0xFF))


def make_read(device_id, address, length):
    params = (address, length)
    body = (device_id, len(params) + 2, 0x02, *params)
    return bytes((0xFF, 0xFF, *body, (~sum(body)) & 0xFF))


def configure_serial(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B1000000
    attrs[5] = termios.B1000000
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def write_all(fd, data):
    offset = 0
    deadline = time.monotonic() + READ_TIMEOUT_SECONDS
    while offset < len(data):
        remaining = max(0.0, deadline - time.monotonic())
        _, writable, _ = select.select([], [fd], [], remaining)
        if not writable:
            raise TimeoutError("serial write timed out")
        offset += os.write(fd, data[offset:])


class SyncReadTimeout(TimeoutError):
    """A timeout retaining the IDs that did not provide a valid frame."""

    def __init__(self, missing_ids):
        self.missing_ids = tuple(missing_ids)
        super().__init__(f"HX SYNC_READ timeout; missing IDs {list(self.missing_ids)}")


def read_blocks(fd, request, expected_ids, block_length=READ_LENGTH,
                timeout_seconds=None, trace=None, servo_status=None):
    """Read validated frames; a callback may retain known servo alarm bits."""
    # Match the production transport: discard stale bytes, write without tcdrain,
    # then let the half-duplex UART switch to receive mode.
    termios.tcflush(fd, termios.TCIFLUSH)
    write_all(fd, request)
    started = time.monotonic()
    expected = set(expected_ids)
    blocks = {}
    parser = FrameParser()
    if trace is not None:
        trace.update(tx=request.hex(), received_ids=[], frames=[], rx_hex="",
                     host_checksum_errors=0, host_invalid_lengths=0)
    deadline = time.monotonic() + (timeout_seconds or READ_TIMEOUT_SECONDS)
    while time.monotonic() < deadline:
        readable, _, _ = select.select(
            [fd], [], [], max(0.0, deadline - time.monotonic())
        )
        if not readable:
            break
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not data:
            continue
        if trace is not None:
            trace["rx_hex"] += data.hex()
        parser.feed(data)
        while True:
            frame = parser.pop()
            if trace is not None:
                trace.update(host_checksum_errors=parser.checksum_errors,
                             host_invalid_lengths=parser.invalid_lengths,
                             pending_hex=parser.buffer.hex())
            if frame is None:
                break
            device_id, error, payload = frame
            if trace is not None:
                trace["frames"].append({"id": device_id, "status": error,
                                        "length": len(payload),
                                        "at_ms": (time.monotonic() - started) * 1000})
            if device_id not in expected or device_id in blocks:
                continue
            if error and (servo_status is None or device_id not in JOINT_IDS
                          or error & 0xC0):
                raise RuntimeError(f"ID {device_id} status error 0x{error:02X}")
            if len(payload) != block_length:
                raise RuntimeError(
                    f"ID {device_id} payload length {len(payload)}, "
                    f"expected {block_length}"
                )
            if servo_status is not None and device_id in JOINT_IDS:
                servo_status(device_id, error)
            blocks[device_id] = payload
            if trace is not None:
                trace["received_ids"].append(device_id)
            if len(blocks) == len(expected_ids):
                return blocks
    raise SyncReadTimeout(device_id for device_id in expected_ids if device_id not in blocks)


def decode_firmware_info(data, expected_id):
    """Decode the firmware's fixed read-only build identity packet."""
    if len(data) != FIRMWARE_INFO_LENGTH or data[:8] != b"IMUV\x01\x00\x40\x00":
        raise ValueError("固件版本接口的标识、格式或长度不匹配")
    major, minor, patch, state, sensor, bus, fusion, timing, device_id = data[8:17]
    accel_lpf, gyro_lpf = data[17:19]
    tau_ms = int.from_bytes(data[20:22], "little")
    sensors = {1: "QMI8658", 2: "LSM6DSV16X"}
    buses = {1: "SPI", 2: "I2C"}
    states = {0: "unknown", 1: "clean", 2: "modified"}
    fusions = {0: "HARDWARE", 1: "GATED", 2: "WEIGHTED", 3: "INERTIAL", 4: "HOST"}
    lpfs = {0: "MODE0", 1: "MODE1", 2: "MODE2", 3: "MODE3", 4: "OFF", 255: None}
    if (state not in states or sensor not in sensors or bus not in buses or fusion not in fusions
            or timing not in (0, 1) or device_id != expected_id
            or accel_lpf not in lpfs or gyro_lpf not in lpfs or data[19] != 0 or data[22:24] != b"\0\0"):
        raise ValueError("固件版本接口返回了无效字段")
    if (sensor == 1 and (fusion == 0 or accel_lpf == 255 or gyro_lpf == 255)) or (
            sensor == 2 and (fusion != 0 or accel_lpf != 255 or gyro_lpf != 255)):
        raise ValueError("固件版本接口的传感器和构建配置不一致")
    if (fusion == 3 and not 100 <= tau_ms <= 5000) or (fusion != 3 and tau_ms != 0):
        raise ValueError("固件版本接口的融合时间常数无效")
    revision_bytes = data[24:64]
    if state == 0:
        if any(revision_bytes):
            raise ValueError("固件源码未知时提交号必须为空")
        revision = None
    else:
        if any(byte not in b"0123456789abcdef" for byte in revision_bytes):
            raise ValueError("固件源码提交号无效")
        revision = revision_bytes.decode("ascii")
    return {
        "device_id": device_id,
        "firmware_version": f"{major}.{minor}.{patch}",
        "source_revision": revision,
        "source_state": states[state],
        "sensor": sensors[sensor],
        "sensor_bus": buses[bus],
        "fusion": fusions[fusion],
        "timing_diagnostics": bool(timing),
        "accel_lpf": lpfs[accel_lpf],
        "gyro_lpf": lpfs[gyro_lpf],
        "accel_tau_ms": tau_ms if fusion == 3 else None,
    }


def decode_gyro_bias(data, *, allow_unloaded_bias=False):
    """Decode MCU fusion readiness and its external fixed-bias capability."""
    if len(data) != 48 or data[:8] != b"IMUB\x02\x00\x30\x00":
        raise ValueError("需要配套的MCU陀螺仪状态接口")
    flags, reserved = struct.unpack_from("<II", data, 8)
    supported = bool(flags & 1)
    loaded = bool(flags & 2)
    ready = bool(flags & 4)
    raw = struct.unpack_from("<3f", data, 16)
    bias = struct.unpack_from("<3f", data, 28)
    age = struct.unpack_from("<I", data, 40)[0]
    if (flags & ~7 or (loaded and not supported)
            or (supported and ready and not loaded)
            or reserved != 0
            or (not loaded and any(bias) and not (allow_unloaded_bias and supported and not ready))
            or data[44:48] != b"QST3" or age > 100000 or not all(math.isfinite(v) for v in raw + bias)
            or math.sqrt(sum(v*v for v in bias)) > GYRO_BIAS_MAX_NORM_RAD_S):
        raise ValueError("MCU陀螺仪状态无效或过期")
    return {"supported": supported, "ready": ready, "loaded": loaded,
            "raw_sensor_rad_s": raw, "bias_sensor_rad_s": bias}


def read_gyro_bias(fd, imu_id=IMU_ID, *, allow_unloaded_bias=False):
    return decode_gyro_bias(read_blocks(
        fd, make_read(imu_id, 122, 48), (imu_id,), block_length=48,
        timeout_seconds=FAULT_TIMEOUT_SECONDS)[imu_id], allow_unloaded_bias=allow_unloaded_bias)


def read_persistent_calibration(fd, imu_id=IMU_ID):
    data = read_blocks(fd, make_read(imu_id, 115, 16), (imu_id,),
                       block_length=16, timeout_seconds=0.2)[imu_id]
    if len(data) != 16 or data[:8] != b"CALP\x01\x00\x10\x00":
        raise ValueError("MCU 不支持持久校准；请先安装配套固件")
    flags, generation = struct.unpack_from("<II", data, 8)
    if flags & ~15 or (flags & 4 and not flags & 2) or (flags & 7 and not generation):
        raise ValueError("MCU 持久校准状态无效")
    return {"flags": flags, "generation": generation}


def apply_fixed_gyro_bias(fd, bias, imu_id=IMU_ID):
    bias = struct.unpack("<3f", struct.pack("<3f", *bias))
    if not all(math.isfinite(v) for v in bias) or math.sqrt(sum(v*v for v in bias)) > GYRO_BIAS_MAX_NORM_RAD_S:
        raise ValueError("固定零偏必须有限且模长不超过0.30rad/s")
    read_persistent_calibration(fd, imu_id)
    state = read_gyro_bias(fd, imu_id, allow_unloaded_bias=True)
    if not state["supported"]:
        raise ValueError("MCU使用硬件融合，不支持加载外部固定零偏")
    payload = struct.pack("<HH3f", 1, 1, *bias)
    body = bytes((imu_id, len(payload) + 3, 3, 122)) + payload
    request = b"\xff\xff" + body + bytes((~sum(body) & 255,))
    replies = read_blocks(fd, request, (imu_id,), block_length=16,
                          timeout_seconds=2.0)
    if replies[imu_id] != payload:
        raise ValueError("固定零偏写入回复与请求不一致")
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        state = read_gyro_bias(fd, imu_id, allow_unloaded_bias=True)
        if state["ready"] and state["loaded"] and tuple(state["bias_sensor_rad_s"]) == bias:
            stored = read_persistent_calibration(fd, imu_id)
            if stored["flags"] & 8 or not stored["flags"] & 2:
                raise ValueError("MCU 未确认永久保存陀螺仪校准")
            return state
        time.sleep(0.005)
    raise ValueError("固定零偏回读或融合就绪超时")



def read_fault_snapshot(fd, imu_id=IMU_ID):
    payload = read_blocks(
        fd, make_read(imu_id, FAULT_ADDRESS, FAULT_LENGTH), (imu_id,),
        block_length=FAULT_LENGTH, timeout_seconds=FAULT_TIMEOUT_SECONDS,
    )[imu_id]
    fields = (
        "request", "reply", "flags", "sample_sequence", "sample_age_us",
        "init_failures", "imu_bus_failures", "imu_fifo_failures",
        "uart_errors", "checksum_errors", "rx_overruns", "main_sequence",
        "reply_us",
    )
    return dict(zip(fields, struct.unpack("<13I", payload)))


def fault_location(snapshot, token):
    if token is None:
        return "unknown_counter_unsynchronized"
    if snapshot["request"] != token:
        return "request_not_received_by_mcu"
    if snapshot["reply"] != token:
        return "mcu_reply_not_completed"
    if snapshot["reply_us"] >= READ_TIMEOUT_SECONDS * 1_000_000:
        return "mcu_reply_after_host_deadline"
    return "mcu_reply_sent_in_time"


def diagnose_imu_failure(fd, token, error, imu_id=IMU_ID):
    try:
        fault = read_fault_snapshot(fd, imu_id)
    except (OSError, RuntimeError, TimeoutError) as probe_error:
        print(f"IMU_LOCATION token={token} location=unknown "
              f"cause={error} probe_error={probe_error}", flush=True)
        return None
    print(
        f"IMU_LOCATION token={token} location={fault_location(fault, token)} "
        f"mcu_request={fault['request']} mcu_reply={fault['reply']} "
        f"imu_ready={bool(fault['flags'] & 1)} "
        f"imu_stale={bool(fault['flags'] & 2)} "
        f"sample_sequence={fault['sample_sequence']} "
        f"sample_age_us={fault['sample_age_us']} "
        f"mcu_reply_us={fault['reply_us']} "
        f"imu_bus_failures={fault['imu_bus_failures']} "
        f"imu_fifo_failures={fault['imu_fifo_failures']} "
        f"uart_errors={fault['uart_errors']} "
        f"checksum_errors={fault['checksum_errors']} "
        f"rx_overruns={fault['rx_overruns']} cause={error}",
        flush=True,
    )
    return fault["request"]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate(q, vector, inverse=False):
    """Rotate a vector using q*v*q^-1, or q^-1*v*q when inverse is true."""
    q_vector = q[1:]
    twice_cross = tuple(2.0 * value for value in cross(q_vector, vector))
    correction = cross(q_vector, twice_cross)
    sign = -1.0 if inverse else 1.0
    return tuple(
        vector[i] + sign * q[0] * twice_cross[i] + correction[i]
        for i in range(3)
    )


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def normalise(vector):
    magnitude = math.sqrt(sum(value * value for value in vector))
    if magnitude > 0.1:
        return tuple(value / magnitude for value in vector)
    return (0.0, 0.0, -1.0)


def median3(a, b, c):
    return max(a, b) if c >= max(a, b) else min(a, b) if c <= min(a, b) else c


class SharedBusDecoder:
    """Current shared-bus smallest-three decoder and host median filter."""

    def __init__(self):
        self.last_quat = (1.0, 0.0, 0.0, 0.0)
        self.quat_samples = 0
        self.gyro_history = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
        self.gravity_history = [(0.0, 0.0, -1.0), (0.0, 0.0, -1.0)]

    @property
    def ready(self):
        return self.quat_samples >= 25

    def decode(self, payload):
        sensor_gyro = tuple(
            value * GYRO_RAD_PER_LSB for value in struct.unpack("<hhh", payload[:6])
        )
        gyro = rotate(HX_MOUNT, sensor_gyro)

        bits = int.from_bytes(payload[6:], "little")
        if bits & (1 << 47):
            raise ValueError("MCU IMU未就绪或四元数无效")
        omitted = (bits >> 45) & 3
        quaternion = [0.0] * 4
        slot = 0
        for axis in range(4):
            if axis == omitted:
                continue
            value = (bits >> (15 * slot)) & 0x7fff
            value = value - 0x8000 if value & 0x4000 else value
            if value == -16384:
                raise ValueError("四元数压缩分量无效")
            quaternion[axis] = value / (16383 * math.sqrt(2))
            slot += 1
        remainder = 1 - sum(value * value for value in quaternion)
        if remainder < .25 - .0001:
            raise ValueError("四元数省略分量无效")
        quaternion[omitted] = math.sqrt(remainder)
        if max(map(abs, quaternion)) > quaternion[omitted] + .0001:
            raise ValueError("四元数没有省略最大分量")
        inverse = (HX_MOUNT[0], -HX_MOUNT[1], -HX_MOUNT[2], -HX_MOUNT[3])
        quaternion = quaternion_multiply(quaternion, inverse)
        magnitude = math.sqrt(sum(value * value for value in quaternion))
        self.last_quat = tuple(value / magnitude for value in quaternion)
        self.quat_samples = min(self.quat_samples + 1, 0xffffffff)

        gravity = normalise(rotate(self.last_quat, (0.0, 0.0, -1.0), inverse=True))
        filtered_gyro = tuple(
            median3(self.gyro_history[0][i], self.gyro_history[1][i], gyro[i])
            for i in range(3)
        )
        filtered_gravity = tuple(
            median3(
                self.gravity_history[0][i],
                self.gravity_history[1][i],
                gravity[i],
            )
            for i in range(3)
        )
        self.gyro_history = [self.gyro_history[1], gyro]
        self.gravity_history = [self.gravity_history[1], gravity]
        return filtered_gyro, filtered_gravity, self.last_quat, sensor_gyro


def format_vector(vector):
    return "[" + ", ".join(f"{value:+.4f}" for value in vector) + "]"


def final_policy_imu(gyro, gravity, freeze_policy_imu):
    """Mirror robotd's final IMU values immediately before ONNX inference."""
    if freeze_policy_imu:
        return (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)
    return gyro, gravity


def signed_magnitude(raw, sign_bit):
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw & (1 << sign_bit) else magnitude


def format_joint_feedback(blocks):
    """Format the position, velocity, and load fields robotd decodes from each joint."""
    values = []
    for device_id in JOINT_IDS:
        block = blocks[device_id]
        position_raw = int.from_bytes(block[:2], "little")
        speed_raw = int.from_bytes(block[2:4], "little")
        load_raw = int.from_bytes(block[4:6], "little")
        position = math.pi - 2.0 * math.pi * position_raw / 4096.0
        velocity = -signed_magnitude(speed_raw, 15) * 2.0 * math.pi / 4096.0
        load = abs(signed_magnitude(load_raw, 10))
        values.append(
            f"{device_id}:pos={position:+.3f}rad,vel={velocity:+.3f}rad/s,load={load}"
        )
    return " joints={" + "; ".join(values) + "}"


def percentile(values, fraction):
    """Return a linearly interpolated percentile without an external dependency."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


class RunStats:
    """Read results and the raw-frame staleness metric used by the runtime."""

    def __init__(self, expected_ids=ALL_DEVICE_IDS):
        self.attempts = 0
        self.successes = 0
        self.timeouts = 0
        self.protocol_errors = 0
        self.os_errors = 0
        self.raw_stale = 0
        self.stale_run = 0
        self.max_stale_run = 0
        self.last_payload = None
        self.read_ms = []
        self.sample_gaps_ms = []
        self.last_started = None
        self.missing_by_id = {device_id: 0 for device_id in expected_ids}

    @property
    def errors(self):
        return self.timeouts + self.protocol_errors + self.os_errors

    def record_start(self, started):
        self.attempts += 1
        if self.last_started is not None:
            self.sample_gaps_ms.append((started - self.last_started) * 1000.0)
        self.last_started = started

    def record_success(self, payload, elapsed_ms):
        self.successes += 1
        self.read_ms.append(elapsed_ms)
        if self.last_payload == payload:
            self.raw_stale += 1
            self.stale_run += 1
            self.max_stale_run = max(self.max_stale_run, self.stale_run)
        else:
            self.stale_run = 0
        self.last_payload = payload

    def record_error(self, error):
        if isinstance(error, TimeoutError):
            self.timeouts += 1
            if isinstance(error, SyncReadTimeout):
                for device_id in error.missing_ids:
                    self.missing_by_id[device_id] += 1
        elif isinstance(error, (RuntimeError, ValueError)):
            self.protocol_errors += 1
        else:
            self.os_errors += 1


def metric_summary(name, values):
    if not values:
        return f"{name}=n/a"
    return (
        f"{name}_mean={sum(values) / len(values):.3f}ms "
        f"{name}_p95={percentile(values, 0.95):.3f}ms "
        f"{name}_max={max(values):.3f}ms"
    )


def print_status(elapsed, stats, decoder, gyro, gravity, policy_gyro, policy_gravity):
    last_read = stats.read_ms[-1] if stats.read_ms else 0.0
    print(
        f"STATUS elapsed={elapsed:.1f}s attempts={stats.attempts} "
        f"ok={stats.successes} errors={stats.errors} "
        f"raw_stale={stats.raw_stale} stale_run={stats.stale_run} "
        f"max_stale_run={stats.max_stale_run} ready={decoder.ready} "
        f"read={last_read:.3f}ms decoded_gyro={format_vector(gyro)} "
        f"decoded_gravity={format_vector(gravity)} "
        f"policy_gyro={format_vector(policy_gyro)} "
        f"policy_imu={format_vector(policy_gravity)}",
        flush=True,
    )


def print_summary(elapsed, stats, args, interrupted):
    failures = []
    if stats.successes < args.min_samples:
        failures.append(f"successes {stats.successes} < min_samples {args.min_samples}")
    if stats.errors > args.max_errors:
        failures.append(f"errors {stats.errors} > max_errors {args.max_errors}")
    if stats.max_stale_run > args.max_stale_run:
        failures.append(
            f"max_stale_run {stats.max_stale_run} > "
            f"max_stale_run {args.max_stale_run}"
        )
    result = "PASS" if not failures else "FAIL"
    missing = ",".join(
        f"{device_id}:{count}"
        for device_id, count in stats.missing_by_id.items()
        if count
    )
    print(
        f"SUMMARY result={result} elapsed={elapsed:.3f}s attempts={stats.attempts} "
        f"ok={stats.successes} errors={stats.errors} timeouts={stats.timeouts} "
        f"protocol_errors={stats.protocol_errors} os_errors={stats.os_errors} "
        f"raw_stale={stats.raw_stale} max_stale_run={stats.max_stale_run} "
        f"{metric_summary('read', stats.read_ms)} "
        f"{metric_summary('sample_gap', stats.sample_gaps_ms)} "
        f"missing_by_id={missing or 'none'} "
        f"interrupted={interrupted}",
        flush=True,
    )
    for failure in failures:
        print(f"FAILURE {failure}", flush=True)
    return 0 if not failures else 1


def read_raw_sensor(fd, imu_id):
    """Read uncorrected sensor-axis values; the two replies are not one snapshot."""
    accel_start = time.monotonic_ns()
    accel = read_blocks(fd, make_read(imu_id, 68, 48), (imu_id,), block_length=48)[imu_id]
    accel_end = time.monotonic_ns()
    schema, model, flags, reserved, sequence, age, *values = struct.unpack("<4BII9f", accel)
    if schema != 1 or model != 1 or flags & ~15 or reserved or not all(map(math.isfinite, values[:3])):
        raise ValueError("invalid raw accelerometer response")
    gyro_start = time.monotonic_ns()
    gyro = read_blocks(fd, make_read(imu_id, 122, 48), (imu_id,), block_length=48)[imu_id]
    gyro_end = time.monotonic_ns()
    if gyro[:8] != b"IMUB\x02\x00\x30\x00" or gyro[44:48] != b"QST3":
        raise ValueError("invalid raw gyroscope response")
    gyro_flags, gyro_reserved = struct.unpack_from("<II", gyro, 8)
    gyro_raw = struct.unpack_from("<3f", gyro, 16)
    gyro_age = struct.unpack_from("<I", gyro, 40)[0]
    if gyro_flags & ~7 or gyro_reserved or not all(map(math.isfinite, gyro_raw)):
        raise ValueError("invalid raw gyroscope status or values")
    return {
        "accel_g": tuple(values[:3]), "gyro_rad_s": gyro_raw,
        "accel_sequence": sequence, "accel_age_us": age,
        "gyro_age_us": gyro_age, "accel_stale": bool(flags & 2),
        "accel_read_start_ns": accel_start, "accel_read_end_ns": accel_end,
        "gyro_read_start_ns": gyro_start, "gyro_read_end_ns": gyro_end,
    }


def run_raw(fd, args):
    expected_ids = (args.imu_id, *JOINT_IDS) if args.all_devices else (args.imu_id,)
    stats = RunStats(expected_ids)
    period = 1.0 / args.hz
    started = next_sample = next_report = time.monotonic()
    interrupted = False
    print(f"START port={args.port} id={args.imu_id} hz={args.hz:.3f} "
          "mode=raw_sensor_axes accel_gyro_same_sample=unconfirmed", flush=True)
    try:
        while True:
            time.sleep(max(0.0, next_sample - time.monotonic()))
            next_sample += period
            tick = time.monotonic()
            stats.record_start(tick)
            try:
                sample = read_raw_sensor(fd, args.imu_id)
                blocks = None
                if args.all_devices:
                    blocks = read_blocks(fd, make_sync_read(expected_ids), expected_ids)
                elapsed_ms = (time.monotonic() - tick) * 1000.0
                stats.record_success(struct.pack("<I", sample["accel_sequence"]), elapsed_ms)
                now = time.monotonic()
                if args.verbose or args.once or now >= next_report:
                    print(f"RAW t={time.time():.3f} accel_sensor_g={format_vector(sample['accel_g'])} "
                          f"gyro_sensor_rad_s={format_vector(sample['gyro_rad_s'])} "
                          f"gyro_sensor_deg_s={format_vector(tuple(math.degrees(v) for v in sample['gyro_rad_s']))} "
                          f"accel_sequence={sample['accel_sequence']} accel_age_us={sample['accel_age_us']} "
                          f"gyro_age_us={sample['gyro_age_us']} accel_stale={sample['accel_stale']} "
                          f"accel_host_ns={sample['accel_read_end_ns']} gyro_host_ns={sample['gyro_read_end_ns']} "
                          f"accel_gyro_same_sample=unconfirmed read_ms={elapsed_ms:.3f}"
                          f"{format_joint_feedback(blocks) if blocks is not None and args.verbose else ''}",
                          flush=True)
                    next_report = now + args.report_interval
            except (OSError, ValueError, RuntimeError, TimeoutError) as error:
                stats.record_error(error)
                print(f"ERROR t={time.time():.3f} {error}", flush=True)
            if args.once:
                break
            if next_sample < time.monotonic() - period:
                next_sample = time.monotonic()
    except KeyboardInterrupt:
        interrupted = True
    return print_summary(time.monotonic() - started, stats, args, interrupted)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read ID200 raw sensor axes or inspect fused IMU data without robotd."
    )
    parser.add_argument("--port", default=PORT_DEFAULT, help="HX serial port")
    parser.add_argument("--id", dest="imu_id", type=int, default=IMU_ID,
                        help="IMU device ID, 0-253 (default: 200)")
    parser.add_argument("--hz", type=float, default=50.0, help="read rate (default: 50)")
    parser.add_argument(
        "--timeout-ms", type=float, default=40.0, help="one read deadline (default: 40)"
    )
    parser.add_argument(
        "--report-interval",
        type=float,
        default=1.0,
        help="seconds between STATUS lines (default: 1)",
    )
    parser.add_argument(
        "--max-errors",
        type=int,
        default=0,
        help="allowed timeout/protocol/serial errors (default: 0)",
    )
    parser.add_argument(
        "--max-stale-run",
        type=int,
        default=24,
        help="allowed consecutive byte-identical raw frames (default: 24)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=1,
        help="minimum successful reads required for PASS (default: 1)",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="print every decoded sample"
    )
    parser.add_argument(
        "--freeze-policy-imu",
        action="store_true",
        help=(
            "select policy view with zero gyro and upright gravity"
        ),
    )
    parser.add_argument("--policy-view", action="store_true",
                        help="read fused policy data instead of ID200 raw sensor axes")
    parser.add_argument(
        "--all-devices",
        action="store_true",
        help=(
            "read the selected IMU and all 15 joints in one broadcast"
        ),
    )
    parser.add_argument(
        "--once", action="store_true", help="read one block and exit"
    )
    parser.add_argument("--firmware-info", action="store_true",
                        help="read firmware version and build information once")
    args = parser.parse_args()
    if args.freeze_policy_imu:
        args.policy_view = True
    if args.policy_view or args.imu_id != IMU_ID:
        args.freeze_policy_imu |= os.environ.get("ROBOT_FREEZE_POLICY_IMU") == "1"
    if not 0 <= args.imu_id <= 253:
        parser.error("--id must be between 0 and 253; 254 is the broadcast address")
    if args.all_devices and args.imu_id in JOINT_IDS:
        parser.error("--id must differ from the joint IDs when using --all-devices")
    if (
        args.hz <= 0.0
        or args.timeout_ms <= 0.0
        or args.report_interval <= 0.0
    ):
        parser.error("--hz, --timeout-ms, and --report-interval must be positive")
    if args.max_errors < 0 or args.max_stale_run < 0 or args.min_samples < 1:
        parser.error("--max-errors and --max-stale-run must be non-negative; --min-samples >= 1")
    return args


def port_is_in_use(port):
    """Refuse competing serial access before sending a bus packet."""
    try:
        result = subprocess.run(["fuser", "-s", port], check=False)
    except FileNotFoundError:
        print("ERROR: fuser is required to verify exclusive serial-port access.")
        return True
    if result.returncode == 0:
        print(f"ERROR: {port} is in use; stop robotd and every other serial client first.")
        return True
    if result.returncode != 1:
        print(f"ERROR: fuser could not inspect {port} (exit {result.returncode}).")
        return True
    return False


def main():
    args = parse_args()
    global READ_TIMEOUT_SECONDS
    READ_TIMEOUT_SECONDS = args.timeout_ms / 1000.0

    service = subprocess.run(
        ["systemctl", "is-active", "--quiet", "robotd.service"],
        check=False,
    )
    if service.returncode == 0:
        print("ERROR: robotd.service is active; stop it before this read-only test.")
        return 2
    if port_is_in_use(args.port):
        return 2

    try:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        print(f"ERROR: cannot open {args.port}: {error}")
        return 2

    expected_ids = (args.imu_id, *JOINT_IDS) if args.all_devices else (args.imu_id,)
    decoder = SharedBusDecoder()
    request = make_sync_read(expected_ids)
    period = 1.0 / args.hz
    next_sample = time.monotonic()
    run_started = next_sample
    next_report = run_started
    stats = RunStats(expected_ids)
    last_gyro = (0.0, 0.0, 0.0)
    last_gravity = (0.0, 0.0, -1.0)
    last_policy_gyro, last_policy_gravity = final_policy_imu(
        last_gyro, last_gravity, args.freeze_policy_imu
    )
    interrupted = False
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        configure_serial(fd)
        if args.firmware_info:
            try:
                data = read_blocks(fd, make_read(args.imu_id, FIRMWARE_INFO_ADDRESS, FIRMWARE_INFO_LENGTH),
                                   (args.imu_id,), block_length=FIRMWARE_INFO_LENGTH)[args.imu_id]
                print(json.dumps(decode_firmware_info(data, args.imu_id), ensure_ascii=False, indent=2))
                return 0
            except (OSError, ValueError, RuntimeError, TimeoutError) as error:
                print(f"ERROR: 固件版本读取失败：{error}；请检查设备及固件是否支持该接口。")
                return 2
        if args.imu_id == IMU_ID and not args.policy_view:
            return run_raw(fd, args)
        try:
            bias_state = read_gyro_bias(fd, args.imu_id)
            if bias_state["supported"]:
                print(f"MCU gyro bias={bias_state}; host subtraction disabled", flush=True)
            else:
                print("MCU hardware fusion; external fixed gyro calibration not required; "
                      "host subtraction disabled", flush=True)
            if not bias_state["ready"]:
                if bias_state["supported"]:
                    raise RuntimeError("MCU固定零偏未就绪；融合数据需要先加载加速度和陀螺标定")
                raise RuntimeError("MCU硬件融合尚未就绪，请检查传感器采样与姿态输出")
            if args.imu_id == IMU_ID and bias_state["supported"]:
                accel = read_blocks(fd, make_read(args.imu_id, 68, 48), (args.imu_id,),
                                    block_length=48)[args.imu_id]
                if accel[0] != 1 or accel[1] != 1 or accel[2] & ~15 or accel[3]:
                    raise ValueError("无效的MCU加速度状态")
                print(f"MCU acceleration calibration applied={bool(accel[2] & 4)}", flush=True)
        except (OSError, ValueError, RuntimeError, TimeoutError) as error:
            print(f"ERROR: MCU IMU status: {error}", flush=True)
            return 2
        try:
            request_token = read_fault_snapshot(fd, args.imu_id)["request"]
        except (OSError, RuntimeError, TimeoutError) as error:
            print(f"ERROR: cannot read MCU fault snapshot: {error}", flush=True)
            return 2
        counter_synced = True
        print(
            f"START port={args.port} ids={list(expected_ids)} hz={args.hz:.3f} "
            f"samples={'one' if args.once else 'until Ctrl+C'} "
            f"timeout={args.timeout_ms:.3f}ms "
            f"policy_imu={'frozen' if args.freeze_policy_imu else 'live'}",
            flush=True,
        )
        while True:
            delay = next_sample - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            next_sample += period
            started = time.monotonic()
            stats.record_start(started)
            try:
                request_token = (request_token + 1) & 0xFFFFFFFF
                try:
                    blocks = read_blocks(fd, request, expected_ids)
                except (OSError, RuntimeError, TimeoutError) as error:
                    observed = diagnose_imu_failure(
                        fd, request_token if counter_synced else None, error,
                        args.imu_id,
                    )
                    if observed is None:
                        counter_synced = False
                    else:
                        request_token = observed
                        counter_synced = True
                    raise
                imu_payload = blocks[args.imu_id]
                gyro, gravity, quaternion, sensor_gyro = decoder.decode(imu_payload)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                stats.record_success(imu_payload, elapsed_ms)
                last_gyro = gyro
                last_gravity = gravity
                last_policy_gyro, last_policy_gravity = final_policy_imu(
                    gyro, gravity, args.freeze_policy_imu
                )
                if args.verbose:
                    print(
                        f"SAMPLE t={time.time():.3f} ready={decoder.ready} "
                        f"read={elapsed_ms:.3f}ms decoded_gyro={format_vector(gyro)} "
                        f"decoded_gravity={format_vector(gravity)} "
                        f"policy_gyro={format_vector(last_policy_gyro)} "
                        f"policy_imu={format_vector(last_policy_gravity)} "
                        f"sensor_gyro={format_vector(sensor_gyro)} "
                        f"quat_wxyz={format_vector(quaternion)}"
                        f"{format_joint_feedback(blocks) if args.all_devices else ''}",
                        flush=True,
                    )
                if args.once:
                    break
            except (OSError, RuntimeError, TimeoutError) as error:
                stats.record_error(error)
                print(f"ERROR t={time.time():.3f} {error}", flush=True)
                if args.once:
                    break
            now = time.monotonic()
            if now >= next_report:
                print_status(
                    now - run_started,
                    stats,
                    decoder,
                    last_gyro,
                    last_gravity,
                    last_policy_gyro,
                    last_policy_gravity,
                )
                next_report = now + args.report_interval
            if next_sample < time.monotonic() - period:
                next_sample = time.monotonic()
    except KeyboardInterrupt:
        interrupted = True
    finally:
        os.close(fd)
    return print_summary(time.monotonic() - run_started, stats, args, interrupted)


if __name__ == "__main__":
    raise SystemExit(main())
