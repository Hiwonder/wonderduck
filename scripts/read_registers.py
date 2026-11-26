"""读取舵机控制参数与内部参考；仅允许READ指令，禁止动作和配置写入。"""

import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import termios
import time


def signed(word, bit):
    magnitude = word & ((1 << bit) - 1)
    return -magnitude if word & (1 << bit) else magnitude


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--helper", type=Path, required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--diagnostics-ids", type=int, nargs='+', default=[])
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("serial_read_helper", args.helper)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    if len(set(args.diagnostics_ids)) != len(args.diagnostics_ids) or any(i not in helper.JOINT_IDS for i in args.diagnostics_ids):
        raise ValueError("诊断ID须不重复且属于实际舵机")
    if helper.port_is_in_use(args.port):
        raise RuntimeError("串口被占用，未发送请求")
    original_write = helper.write_all
    requests = []

    def only_read(fd, packet):
        if (len(packet) != 8 or packet[:2] != b"\xff\xff"
                or packet[3] != 4 or packet[4] != 2
                or sum(packet[2:]) & 255 != 255):
            raise ValueError("阻止非READ或格式异常的串口请求")
        requests.append({"id": packet[2], "address": packet[5], "length": packet[6]})
        return original_write(fd, packet)

    helper.write_all = only_read
    with args.output.open("x") as stream:
        def emit(kind, **values):
            stream.write(json.dumps({"type": kind, **values}, allow_nan=False) + "\n")
            stream.flush()

        emit("header", unix_time_s=time.time(), port=args.port, hardware_writes=0,
             source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in (Path(__file__), args.helper)})
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            helper.configure_serial(fd)
            for device in helper.JOINT_IDS:
                def read(address, length):
                    return helper.read_blocks(
                        fd, helper.make_read(device, address, length), (device,),
                        block_length=length)[device]

                version = read(0, 5)
                started = time.monotonic_ns()
                config = read(21, 29)
                state = read(56, 15)
                def word(data, offset):
                    return int.from_bytes(data[offset:offset + 2], "little")

                emit("servo", id=device, request_start_ns=started,
                     read_end_ns=time.monotonic_ns(), version_bytes=list(version),
                     p=config[0], d=config[1], i=config[2],
                     minimum_pwm=word(config, 3), deadzone_cw=config[5],
                     deadzone_ccw=config[6], mode=config[12],
                     torque_enable=config[19], acceleration=config[20],
                     final_goal_counts=signed(word(config, 21), 15),
                     duration_ms=word(config, 23), goal_speed=word(config, 25),
                     maximum_pwm=word(config, 27),
                     position_counts=signed(word(state, 0), 15),
                     velocity_counts_s=signed(word(state, 2), 15),
                     pwm_signed=signed(word(state, 4), 10),
                     voltage_v=state[6] / 10, temperature_c=state[7],
                     status=state[9], is_moving=state[10],
                     internal_goal_counts=signed(word(state, 11), 15),
                     current_register=word(state, 13),
                     raw_config=list(config), raw_state=list(state))
                if device in args.diagnostics_ids:
                    begin = time.monotonic_ns()
                    diagnostics = read(0x80, 12)
                    emit("servo_diagnostics_baseline", id=device,
                         read_start_t_ns=begin, read_end_t_ns=time.monotonic_ns(),
                         isr_max_us=int.from_bytes(diagnostics[0:4], 'little'),
                         isr_overrun_count=int.from_bytes(diagnostics[4:8], 'little'),
                         sensor_read_fail_count=int.from_bytes(diagnostics[8:12], 'little'),
                         raw=list(diagnostics), support_confirmed=False)
            emit("completion", serial_read_requests=len(requests), requests=requests,
                 hardware_writes=0, servo_count=len(helper.JOINT_IDS))
        except (OSError, ValueError, RuntimeError, TimeoutError) as error:
            emit("error", message=str(error), serial_read_requests=len(requests),
                 hardware_writes=0)
            raise
        finally:
            os.close(fd)
    print(json.dumps({"output": str(args.output), "read_requests": len(requests),
                      "hardware_writes": 0}))


if __name__ == "__main__":
    main()
