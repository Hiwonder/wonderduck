#!/usr/bin/env python3
"""Read HX servo feedback without sending motion or torque commands."""

import argparse
import fcntl
import importlib.util
import math
import os
from pathlib import Path
import select
import struct
import termios
import time


SPEC = importlib.util.spec_from_file_location(
    "imu_live", Path(__file__).with_name("imu-live.py")
)
LIVE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LIVE)


def make_request(ids, mode):
    if mode == "sync":
        return LIVE.make_sync_read(ids)
    if len(ids) != 1:
        raise ValueError("单播 READ 每次只能读取一个 ID")
    body = (ids[0], 4, 0x02, LIVE.READ_ADDRESS, LIVE.READ_LENGTH)
    return bytes((255, 255, *body, (~sum(body)) & 255))


def read_feedback(fd, ids, mode, timeout_seconds):
    """Keep partial replies and raw RX evidence when another device is missing."""
    request = make_request(ids, mode)
    termios.tcflush(fd, termios.TCIFLUSH)
    started = time.monotonic()
    LIVE.write_all(fd, request)
    deadline = time.monotonic() + timeout_seconds
    parser = LIVE.FrameParser()
    blocks, faults = {}, {}
    raw_head = bytearray()
    rx_bytes = 0
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], max(0., deadline - time.monotonic()))
        if not ready:
            break
        try:
            data = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not data:
            raise OSError("串口读取返回 EOF")
        rx_bytes += len(data)
        raw_head.extend(data[:max(0, 32 - len(raw_head))])
        parser.feed(data)
        while True:
            frame = parser.pop()
            if frame is None:
                break
            device_id, error, payload = frame
            if device_id not in ids or device_id in blocks or device_id in faults:
                continue
            if error:
                faults[device_id] = f"status=0x{error:02X}"
            if len(payload) != LIVE.READ_LENGTH:
                if error:
                    continue
                faults[device_id] = f"length={len(payload)} expected={LIVE.READ_LENGTH}"
            else:
                blocks[device_id] = payload
        if len(set(blocks) | set(faults)) == len(ids):
            break
    missing = [device_id for device_id in ids
               if device_id not in blocks and device_id not in faults]
    elapsed_ms = (time.monotonic() - started) * 1000.
    if missing or faults:
        print(f"ERROR ids={list(ids)} missing={missing} faults={faults} "
              f"rx_bytes={rx_bytes:4d} read_ms={elapsed_ms:7.2f} "
              f"tx={request.hex()} rx_head={raw_head.hex() or '-'}", flush=True)
    return blocks, faults, missing, rx_bytes


def decode_feedback(payload):
    """Native servo units; angle is encoder angle, not robot joint angle."""
    position, speed, load, voltage, temperature = struct.unpack_from("<HHHBB", payload)
    return (position, position * 360. / 4096., LIVE.signed_magnitude(speed, 15),
            LIVE.signed_magnitude(load, 10) / 10., voltage / 10., temperature)


def print_feedback(ids, blocks, faults, read_times=None):
    print(" ID  state     raw  angle_deg  speed_step/s   load_%      V    temp_C")
    for device_id in ids:
        elapsed = (f"  read_ms={read_times[device_id]:.2f}"
                   if read_times is not None else "")
        if device_id in faults and device_id not in blocks:
            print(f"{device_id:3d}  ERROR   {faults[device_id]}{elapsed}")
        elif device_id not in blocks:
            print(f"{device_id:3d}  MISSING{elapsed}")
        else:
            raw, angle, speed, load, voltage, temperature = decode_feedback(blocks[device_id])
            print(f"{device_id:3d}  OK     {raw:5d}  {angle:9.2f}  {speed:12d} "
                  f"{load:8.2f}  {voltage:5.2f}  {temperature:8.2f}"
                  + elapsed + (f"  告警 {faults[device_id]}" if device_id in faults else ""))


def parse_args(argv=None, default_mode="sync"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=LIVE.PORT_DEFAULT)
    parser.add_argument("--ids", type=int, nargs="+", default=LIVE.JOINT_IDS,
                        help="舵机 ID 列表，默认全部 15 个关节；不包含 IMU ID 200")
    parser.add_argument("--mode", choices=("sync", "read"), default=default_mode,
                        help=f"sync: 一次广播；read: 按 ID 逐个单播；默认{default_mode}")
    parser.add_argument("--hz", type=float, default=50., help="每秒读取轮数")
    parser.add_argument("--timeout-ms", type=float, default=40., help="每个请求接收超时")
    parser.add_argument("--report-interval", type=float, default=1., help="数据打印间隔/秒")
    parser.add_argument("--once", action="store_true", help="只读一轮并退出")
    args = parser.parse_args(argv)
    if (len(set(args.ids)) != len(args.ids)
            or any(i < 0 or i > 253 or i == LIVE.IMU_ID for i in args.ids)):
        parser.error("ID 必须唯一且在 0..253，不能包含 IMU ID 200")
    if any(not math.isfinite(v) or v <= 0
           for v in (args.hz, args.timeout_ms, args.report_interval)):
        parser.error("--hz、--timeout-ms、--report-interval 必须为有限正数")
    return args


def monitor(fd, args):
    groups = [tuple(args.ids)] if args.mode == "sync" else [(i,) for i in args.ids]
    counts = {i: [0, 0, 0] for i in args.ids}
    started = next_sample = next_report = time.monotonic()
    cycles = failures = 0
    print(f"START port={args.port} baud=1000000 mode={args.mode} ids={list(args.ids)} "
          f"hz={args.hz:.2f} timeout_ms={args.timeout_ms:.2f}", flush=True)
    try:
        while True:
            time.sleep(max(0., next_sample - time.monotonic()))
            cycle_start = time.monotonic()
            blocks, faults, rx_bytes = {}, {}, 0
            read_times = {}
            for group in groups:
                read_started = time.monotonic()
                good, bad, missing, received = read_feedback(
                    fd, group, args.mode, args.timeout_ms / 1000.
                )
                blocks.update(good)
                faults.update(bad)
                rx_bytes += received
                for i in group:
                    read_times[i] = (time.monotonic() - read_started) * 1000.
                    counts[i][0 if i in good else 1 if i in missing else 2] += 1
            cycles += 1
            failures += int(len(blocks) != len(args.ids))
            now = time.monotonic()
            if now >= next_report or args.once:
                print(f"STATUS t={now - started:7.2f}s cycles={cycles} failures={failures} "
                      f"ok={len(blocks):2d}/{len(args.ids)} rx_bytes={rx_bytes:4d} "
                      f"read_ms={(now - cycle_start) * 1000.:7.2f}")
                print_feedback(args.ids, blocks, faults,
                               read_times if args.mode == 'read' else None)
                print(flush=True)
                next_report = now + args.report_interval
            if args.once:
                break
            # A timeout must not trigger a burst of catch-up requests.
            next_sample = max(next_sample + 1. / args.hz, time.monotonic())
    except KeyboardInterrupt:
        print("\nSTOP", flush=True)
    finally:
        print("SUMMARY\n ID       ok  missing    error")
        for i, (ok, missing, error) in counts.items():
            print(f"{i:3d}  {ok:7d}  {missing:7d}  {error:7d}")
    return 0 if cycles and not failures else 1


def main(argv=None, default_mode="sync"):
    args = parse_args(argv, default_mode)
    if LIVE.port_is_in_use(args.port):
        return 2
    LIVE.READ_TIMEOUT_SECONDS = args.timeout_ms / 1000.
    try:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            LIVE.configure_serial(fd)
            return monitor(fd, args)
        finally:
            os.close(fd)
    except (OSError, TimeoutError) as error:
        print(f"ERROR: {error}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
