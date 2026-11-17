#!/usr/bin/env python3
"""Publish single-zone distance frames for the local robot control daemon."""

import argparse
from collections import deque
import json
import math
import statistics
import sys
import time


MIN_DISTANCE_MM = 20
MAX_DISTANCE_MM = 2500
POLL_INTERVAL_SECONDS = 0.003


def distance_is_valid(value):
    return isinstance(value, int) and MIN_DISTANCE_MM <= value <= MAX_DISTANCE_MM


def configure_sensor(sensor, timing_budget_ms=None, signal_rate_limit=None):
    if timing_budget_ms is not None:
        sensor.measurement_timing_budget = timing_budget_ms * 1000
    if signal_rate_limit is not None:
        sensor.signal_rate_limit = signal_rate_limit


def throttled_sensor_class(base):
    """Yield between unsuccessful driver polls without changing ranging mode."""
    class ThrottledVL53L0X(base):
        _poll_register = None

        def _read_u8(self, register):
            value = super()._read_u8(register)
            if register == self._poll_register and (
                register == 0x83 and value == 0
                or register == 0x13 and value & 0x07 == 0
                or register == 0x00 and value & 0x01
            ):
                time.sleep(POLL_INTERVAL_SECONDS)
            return value

        def _polling(self, register, function, *args):
            previous = self._poll_register
            self._poll_register = register
            try:
                return function(*args)
            finally:
                self._poll_register = previous

        def _get_spad_info(self):
            return self._polling(0x83, super()._get_spad_info)

        def _perform_single_ref_calibration(self, vhv_init_byte):
            return self._polling(0x13, super()._perform_single_ref_calibration, vhv_init_byte)

        def do_range_measurement(self):
            return self._polling(0x00, super().do_range_measurement)

        def read_range(self):
            return self._polling(0x13, super().read_range)

    return ThrottledVL53L0X


class DistanceFilter:
    def __init__(self):
        self.samples = deque(maxlen=3)

    def update(self, distance):
        if not distance_is_valid(distance):
            self.samples.clear()
            return None
        self.samples.append(distance)
        return statistics.median(self.samples) if len(self.samples) == 3 else None


def sample_status(raw, filtered):
    if not isinstance(raw, int) or raw <= 0 or raw in (8190, 8191, 65535):
        return "invalid"
    if not distance_is_valid(raw):
        return "out_of_range"
    return "valid" if filtered is not None else "filtering"


def frame(distance, sensor_status, raw_distance=None, include_raw=False):
    result = {
        "distance_mm": distance if distance_is_valid(distance) else None,
        "sensor_status": sensor_status,
        "sampled_monotonic_ns": time.monotonic_ns(),
    }
    if include_raw:
        result["raw_distance_mm"] = raw_distance if isinstance(raw_distance, int) and 0 <= raw_distance <= 65535 else None
    return result


def publish(distance, sensor_status, raw_distance=None, include_raw=False):
    print(json.dumps(frame(distance, sensor_status, raw_distance, include_raw), separators=(",", ":")), flush=True)


def restore_identity_page(bus):
    """Return the configured VL53L0X to its normal register page after interrupted access."""
    if not bus.try_lock():
        raise OSError("I2C bus busy while restoring sensor register page")
    try:
        bus.writeto(0x29, bytes((0xff, 0x00)))
    finally:
        bus.unlock()


def run(bus_number, hz, timing_budget_ms=None, signal_rate_limit=None, include_raw=False):
    try:
        import adafruit_vl53l0x
        from adafruit_extended_bus import ExtendedI2C
    except ImportError as error:
        print(f"distance sensor dependencies unavailable: {error}", file=sys.stderr, flush=True)
        return 1

    sensor_class = throttled_sensor_class(adafruit_vl53l0x.VL53L0X)
    reported_error = False
    page_reset_attempted = False
    reset_error_reported = False
    while True:
        bus = None
        try:
            bus = ExtendedI2C(bus_number)
            sensor = sensor_class(bus, address=0x29, io_timeout_s=1.0)
            configure_sensor(sensor, timing_budget_ms, signal_rate_limit)
            page_reset_attempted = False
            reset_error_reported = False
            distance_filter = DistanceFilter()
            if reported_error:
                print("distance sensor recovered", file=sys.stderr, flush=True)
                reported_error = False
            next_sample = time.monotonic()
            report_start = next_sample
            sample_count = 0
            invalid_count = 0
            last_invalid = None
            invalid_reported = False
            while True:
                time.sleep(max(0.0, next_sample - time.monotonic()))
                raw = sensor.range
                filtered = distance_filter.update(raw)
                status = sample_status(raw, filtered)
                publish(filtered, status, raw, include_raw)
                now = time.monotonic()
                sample_count += 1
                if status == "invalid":
                    invalid_count += 1
                    last_invalid = raw
                if now - report_start >= 10.0:
                    if invalid_count and not invalid_reported:
                        print(f"distance sensor invalid readings: {invalid_count}/{sample_count}, "
                              f"last_raw={last_invalid!r}", file=sys.stderr, flush=True)
                    elif not invalid_count and invalid_reported:
                        print("distance sensor invalid readings recovered", file=sys.stderr, flush=True)
                    invalid_reported = bool(invalid_count)
                    report_start = now
                    sample_count = invalid_count = 0
                next_sample = max(next_sample + 1.0 / hz, now)
        except (OSError, RuntimeError, ValueError) as error:
            publish(None, "error" if isinstance(error, ValueError) else "unavailable", include_raw=include_raw)
            if not reported_error:
                print(f"distance sensor unavailable: {error}", file=sys.stderr, flush=True)
                reported_error = True
            if (bus is not None and not page_reset_attempted and isinstance(error, RuntimeError)
                    and str(error) == "Failed to find expected ID register values. Check wiring!"):
                try:
                    restore_identity_page(bus)
                    page_reset_attempted = True
                    reset_error_reported = False
                    print("distance sensor register page restored; retrying identification", file=sys.stderr, flush=True)
                except OSError as reset_error:
                    if not reset_error_reported:
                        print(f"distance sensor register page reset failed: {reset_error}", file=sys.stderr, flush=True)
                        reset_error_reported = True
        finally:
            if bus is not None:
                bus.deinit()
        time.sleep(5.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=5)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--timing-budget-ms", type=int)
    parser.add_argument("--signal-rate-limit", type=float)
    parser.add_argument("--include-raw", action="store_true")
    args = parser.parse_args()
    if args.bus < 0 or not math.isfinite(args.hz) or not 1 <= args.hz <= 20:
        parser.error("bus must be nonnegative and sampling rate must be 1..20 Hz")
    if args.timing_budget_ms is not None and not 20 <= args.timing_budget_ms <= 500:
        parser.error("timing budget must be 20..500 ms")
    if args.signal_rate_limit is not None and (not math.isfinite(args.signal_rate_limit)
            or not 0.01 <= args.signal_rate_limit <= 1.0):
        parser.error("signal rate limit must be 0.01..1.0 MCPS")
    return run(args.bus, args.hz, args.timing_budget_ms, args.signal_rate_limit, args.include_raw)


if __name__ == "__main__":
    sys.exit(main())
