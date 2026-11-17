#!/usr/bin/env python3
"""Play startup and low-battery prompts without owning the motor bus."""

import ast
import fcntl
import http.client
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent.parent
SOUNDS = ROOT / "target/prompt-sounds"
PLAYER = ROOT / "scripts/audio-output.py"
OUTPUT_LOCK = Path("/run/lock/audio-output.lock")
CONFIG = Path("/etc/robot/robotd.toml")
SOCKET = Path("/run/robotd.sock")
STUDIO_HOST = "127.0.0.1"
STUDIO_PORT = 8090
SERVO_IDS = frozenset((20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14))
STARTUP_MARKER = Path("/run/wonderduck-startup-sound.played")
LOW_MARKER = Path("/var/lib/robot/prompt-low-battery.active")
TIMING_MARKER = Path("/run/wonderduck-low-battery-timing.json")
VOLUME_MARKER = Path("/var/lib/robot/prompt-volume.json")
THRESHOLD_V = 7.0
LOW_DELAY_MS = 1000
POLL_INTERVAL = 0.5
PROMPT_INTERVAL = 5.0
BUSY_RETRY = 0.5
PLAYER_TIMEOUT = 5.0
PROMPT_WARMUP_MS = 500
LOG_INTERVAL = 60.0
VOLUME_RETRY = 5.0
EX_TEMPFAIL = 75


def audio_settings(path):
    """Read the existing audio device and master switch without changing the mixer."""
    device, enabled, section = "plughw:aic3104", True, ""
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return device, enabled
    for line in lines:
        line = line.strip()
        if line.startswith("["):
            section = line.split("#", 1)[0].strip()
        elif section == "[audio]" and "=" in line and not line.startswith("#"):
            name, value = line.split("=", 1)
            name, value = name.strip(), value.strip()
            if name == "enabled":
                value = value.split("#", 1)[0].strip()
                if value in ("true", "false"):
                    enabled = value == "true"
            elif name == "device":
                try:
                    parsed = ast.literal_eval(value)
                    if isinstance(parsed, str) and parsed and "\x00" not in parsed:
                        device = parsed
                except (SyntaxError, ValueError):
                    pass
    return device, enabled


def claim_startup(path):
    """One startup attempt per system boot, including across service restarts."""
    try:
        descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError:
        return False
    os.close(descriptor)
    return True


def warning_marker_state(path):
    """An unreadable marker is unknown, so retain the warning conservatively."""
    try:
        existed = path.parent.exists()
        path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        if not existed:
            descriptor = os.open(str(path.parent.parent), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    except OSError:
        return True, False
    try:
        path.lstat()
        return True, True
    except FileNotFoundError:
        return False, True
    except OSError:
        return True, False


def acquire_speaker(path):
    """Acquire the shared output lock without interrupting another sound."""
    try:
        descriptor = os.open(str(path), os.O_RDONLY)
    except FileNotFoundError:
        try:
            descriptor = os.open(str(path), os.O_RDONLY | os.O_CREAT | os.O_EXCL, 0o666)
        except FileExistsError:
            descriptor = os.open(str(path), os.O_RDONLY)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return descriptor
    except BlockingIOError:
        os.close(descriptor)
        return None
    except BaseException:
        os.close(descriptor)
        raise


def speaker_available(path):
    descriptor = acquire_speaker(path)
    if descriptor is None:
        return False
    os.close(descriptor)
    return True


def read_voltage(path):
    """Use robotd's fresh protection observation, never a cached battery average."""
    request = b'{"jsonrpc":"2.0","id":1,"method":"robot.lowVoltageStatus"}\n'
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(0.35)
            connection.connect(str(path))
            connection.sendall(request)
            reply = bytearray()
            while b"\n" not in reply and len(reply) <= 65536:
                block = connection.recv(4096)
                if not block:
                    return None
                reply.extend(block)
        if len(reply) > 65536:
            return None
        message = json.loads(reply.split(b"\n", 1)[0])
        if not isinstance(message, dict) or message.get("id") != 1 or message.get("error") is not None:
            return None
        result = message.get("result")
        if not isinstance(result, dict):
            return None
        settings = result.get("settings")
        if (not isinstance(settings, dict)
                or type(settings.get("threshold_v")) not in (int, float)
                or settings["threshold_v"] != THRESHOLD_V
                or type(settings.get("duration_s")) is not int
                or type(result.get("triggered")) is not bool):
            return None
        voltage = result.get("voltage_v")
        low_for = result.get("low_for_ms")
        if type(voltage) not in (int, float) or not math.isfinite(voltage) or voltage <= 0:
            # A triggered protection has already verified 30 seconds of low
            # voltage, even if its last sample has since become stale.
            return (None, None, True) if result["triggered"] else None
        if voltage >= THRESHOLD_V:
            if low_for is not None and not result["triggered"]:
                return None
            return float(voltage), None, result["triggered"]
        if type(low_for) is not int or low_for < 0:
            return (float(voltage), None, True) if result["triggered"] else None
        return float(voltage), low_for, result["triggered"]
    except (OSError, ValueError, TypeError, OverflowError):
        return None


def read_maintenance_snapshot(host=STUDIO_HOST, port=STUDIO_PORT):
    """Fetch a cached Studio snapshot, without a workspace or sensor request."""
    connection = http.client.HTTPConnection(host, port, timeout=.25)
    try:
        connection.request("GET", "/api/internal/maintenance-voltage", headers={"Connection": "close"})
        response = connection.getresponse()
        if response.status != 200:
            return None
        body = response.read(16385)
        if len(body) > 16384:
            return None
        return json.loads(body)
    except (OSError, ValueError, http.client.HTTPException):
        return None
    finally:
        connection.close()


class MaintenanceVoltage:
    """Debounce only complete, fresh and advancing maintenance snapshots."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_sequence = None
        self.last_sample = None
        self.low_since = None

    def observe(self, snapshot, now):
        try:
            if not isinstance(snapshot, dict) or snapshot.get("available") is not True:
                self.reset()
                return None
            rows, sequence = snapshot.get("servos"), snapshot.get("sequence")
            if not isinstance(rows, list) or len(rows) != len(SERVO_IDS) or type(sequence) is not int or sequence <= 0:
                raise ValueError("invalid snapshot")
            by_id, sampled = {}, []
            for row in rows:
                if not isinstance(row, dict):
                    raise ValueError("invalid servo")
                device_id, voltage, stamp = row.get("id"), row.get("voltage"), row.get("sampled_monotonic")
                if (type(device_id) is not int or device_id not in SERVO_IDS or device_id in by_id
                        or type(voltage) not in (int, float) or not math.isfinite(voltage) or not 0 < voltage <= 25.5
                        or type(stamp) not in (int, float) or not math.isfinite(stamp)
                        or not 0 <= now - stamp <= .25):
                    raise ValueError("invalid servo observation")
                by_id[device_id] = float(voltage)
                sampled.append(float(stamp))
            if set(by_id) != SERVO_IDS:
                raise ValueError("incomplete servo set")
            acquired = min(sampled)
            if self.last_sequence is not None:
                if sequence <= self.last_sequence or acquired <= self.last_sample:
                    self.reset()
                    return None
                if acquired - self.last_sample > .8:
                    self.low_since = None
            self.last_sequence, self.last_sample = sequence, acquired
            voltage = min(by_id.values())
            if voltage >= THRESHOLD_V:
                self.low_since = None
                return voltage, None, False
            if self.low_since is None:
                self.low_since = acquired
            return voltage, max(0, int((acquired - self.low_since) * 1000)), False
        except (ValueError, TypeError, OverflowError):
            self.reset()
            return None


class PromptVolume:
    """Temporarily override the same mixer controls used by the web volume slider."""
    def __init__(self, device, marker=VOLUME_MARKER):
        prefix = "plughw:" if device.startswith("plughw:") else "hw:" if device.startswith("hw:") else None
        value = device[len(prefix):].split(",", 1)[0] if prefix else ""
        if value.startswith("CARD="):
            value = value[5:]
        if not re.fullmatch(r"[A-Za-z0-9_-]+", value):
            raise ValueError("Cannot identify the speaker sound card")
        self.card, self.marker, self.saved, self.error = value, marker, None, None
        try:
            with marker.open("r", encoding="utf-8") as stream:
                self.saved = self._validated(json.load(stream))
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError) as error:
            self.error = "Cannot read saved prompt volume: {}".format(error)

    def _validated(self, value):
        if not isinstance(value, dict) or set(value) != {"card", "min", "max", "pcm", "switch"}:
            raise ValueError("invalid saved volume")
        low, high, pcm, switch = value["min"], value["max"], value["pcm"], value["switch"]
        if (value["card"] != self.card or type(low) is not int or type(high) is not int or high <= low
                or not isinstance(pcm, list) or not 1 <= len(pcm) <= 8
                or any(type(item) is not int or not low <= item <= high for item in pcm)
                or not isinstance(switch, list) or len(switch) != len(pcm)
                or any(item not in ("on", "off") for item in switch)):
            raise ValueError("invalid saved mixer values")
        return value

    def mixer(self, *arguments):
        result = subprocess.run(["amixer", "-c", self.card, *arguments], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, timeout=2,
                                env=dict(os.environ, LC_ALL="C"))
        if result.returncode:
            raise RuntimeError("Cannot adjust prompt volume: {}".format(result.stderr.strip()))
        return result.stdout

    def read_values(self, name):
        output = self.mixer("cget", "name=" + name)
        values = next((line.strip()[len(": values="):] for line in output.splitlines()
                       if line.strip().startswith(": values=")), None)
        if values is None:
            raise ValueError("Cannot read speaker mixer values")
        return output, [item.strip() for item in values.split(",")]

    def snapshot(self):
        output, pcm = self.read_values("PCM Playback Volume")
        _, switches = self.read_values("Line Playback Switch")
        limits = re.search(r"\bmin=(-?\d+),max=(-?\d+)", output)
        if limits is None:
            raise ValueError("Cannot read speaker volume range")
        return self._validated({"card": self.card, "min": int(limits.group(1)), "max": int(limits.group(2)),
                                "pcm": [int(item) for item in pcm], "switch": switches})

    def maximize(self):
        if self.error:
            raise RuntimeError(self.error)
        if self.saved is None:
            saved = self.snapshot()
            temporary = self.marker.with_name("." + self.marker.name + ".tmp")
            descriptor = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(saved, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(str(temporary), str(self.marker))
            directory = os.open(str(self.marker.parent), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
            self.saved = saved
        try:
            self.mixer("cset", "name=PCM Playback Volume", ",".join([str(self.saved["max"])] * len(self.saved["pcm"])))
            self.mixer("cset", "name=Line Playback Switch", ",".join(["on"] * len(self.saved["switch"])))
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError):
            try:
                self.restore()
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError):
                pass
            raise

    def restore(self):
        if self.error:
            raise RuntimeError(self.error)
        if self.saved is None:
            return
        # Mute during restoration so a previously muted speaker never emits a burst.
        self.mixer("cset", "name=Line Playback Switch", ",".join(["off"] * len(self.saved["switch"])))
        self.mixer("cset", "name=PCM Playback Volume", ",".join(str(value) for value in self.saved["pcm"]))
        self.mixer("cset", "name=Line Playback Switch", ",".join(self.saved["switch"]))
        self.marker.unlink(missing_ok=True)
        directory = os.open(str(self.marker.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        self.saved = None


def prompt_threshold():
    value = os.environ.get("WONDERDUCK_PROMPT_TEST_THRESHOLD_V")
    if value is None:
        return THRESHOLD_V
    try:
        threshold = float(value)
        if not math.isfinite(threshold) or not 0 < threshold <= 25.5:
            raise ValueError("out of range")
        return threshold
    except ValueError:
        print("Invalid temporary prompt voltage threshold; using 7.0 V", file=sys.stderr, flush=True)
        return THRESHOLD_V


class WarningState:
    def __init__(self, active=False, threshold=THRESHOLD_V):
        self.active = active
        self.threshold = threshold
        self.low_since = None
        self.last_low_sample = None

    def observe(self, sample, now=None):
        """An unknown sample never clears an already active warning."""
        previous = self.active
        now = time.monotonic() if now is None else now
        if sample is not None:
            voltage, low_for_ms, triggered = sample
            if voltage is not None and voltage >= self.threshold:
                self.active = False
                self.low_since = self.last_low_sample = None
            elif voltage is None and triggered:
                self.active = True
            elif low_for_ms is not None and low_for_ms >= LOW_DELAY_MS:
                self.active = True
            elif voltage is not None and voltage < self.threshold and self.threshold != THRESHOLD_V:
                if self.last_low_sample is None or now - self.last_low_sample > POLL_INTERVAL * 3:
                    self.low_since = now
                self.last_low_sample = now
                if now - self.low_since >= LOW_DELAY_MS / 1000:
                    self.active = True
        else:
            self.low_since = self.last_low_sample = None
        return self.active != previous


class PromptService:
    def __init__(self, socket_path=SOCKET, sound_dir=SOUNDS, player=PLAYER,
                 startup_marker=STARTUP_MARKER, low_marker=LOW_MARKER,
                 timing_marker=TIMING_MARKER, config=CONFIG, maintenance_reader=None,
                 volume_marker=VOLUME_MARKER):
        self.socket_path, self.sound_dir, self.player = socket_path, sound_dir, player
        self.startup_marker, self.low_marker, self.timing_marker = startup_marker, low_marker, timing_marker
        self.device, self.audio_enabled = audio_settings(config)
        active, self.warning_saved = warning_marker_state(low_marker)
        self.warning = WarningState(active, prompt_threshold())
        volume_error = None
        try:
            self.volume = PromptVolume(self.device, volume_marker)
        except ValueError as error:
            self.volume = None
            volume_error = error
        self.low_volume_asserted = False
        self.next_volume_retry = 0.0
        self.maintenance_voltage = MaintenanceVoltage()
        self.maintenance_reader = maintenance_reader or read_maintenance_snapshot
        self.last_warning_attempt = None
        if self.warning.active:
            try:
                saved = json.loads(timing_marker.read_text(encoding="utf-8"))
                last = saved.get("last_attempt")
                if (type(last) in (int, float) and math.isfinite(last)
                        and 0 <= last <= time.monotonic()):
                    self.last_warning_attempt = float(last)
            except (OSError, ValueError, TypeError, AttributeError):
                pass
        startup_error = None
        try:
            self.startup_pending = claim_startup(startup_marker) and self.audio_enabled
        except OSError as error:
            self.startup_pending = False
            startup_error = error
        self.startup_deadline = time.monotonic() + 30.0
        self.child = None
        self.child_kind = None
        self.child_started = 0.0
        self.next_poll = 0.0
        self.next_startup = 0.0
        self.next_warning = (0.0 if self.last_warning_attempt is None
                             else self.last_warning_attempt + PROMPT_INTERVAL)
        self.last_error = -LOG_INTERVAL
        self.stopping = False
        if startup_error is not None:
            self.warn("Cannot save startup sound state: {}".format(startup_error), time.monotonic())
        if volume_error is not None:
            self.warn("Cannot prepare prompt volume: {}".format(volume_error), time.monotonic())
        if self.warning.threshold != THRESHOLD_V:
            print("Temporary low-battery prompt threshold: {:.2f} V".format(self.warning.threshold),
                  file=sys.stderr, flush=True)

    def warn(self, message, now):
        if now - self.last_error >= LOG_INTERVAL:
            print(message, file=sys.stderr, flush=True)
            self.last_error = now

    def sync_warning_marker(self, now):
        try:
            if self.warning.active:
                temporary = self.low_marker.with_name("." + self.low_marker.name + ".tmp")
                self.low_marker.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
                descriptor = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
                with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                    stream.write("low\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(str(temporary), str(self.low_marker))
            else:
                self.low_marker.unlink(missing_ok=True)
            descriptor = os.open(str(self.low_marker.parent), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            self.warning_saved = True
            return True
        except OSError as error:
            self.warn("Cannot save battery warning state: {}".format(error), now)
            self.warning_saved = False
            return False

    def sync_timing_marker(self, now):
        try:
            temporary = self.timing_marker.with_name("." + self.timing_marker.name + ".tmp")
            descriptor = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump({"last_attempt": self.last_warning_attempt}, stream)
            os.replace(str(temporary), str(self.timing_marker))
        except OSError as error:
            self.warn("Cannot save prompt timing: {}".format(error), now)

    def maximize_volume(self, now):
        if self.volume is None:
            return False
        try:
            self.volume.maximize()
            return True
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            self.warn("Cannot maximize prompt volume: {}".format(error), now)
            return False

    def sync_volume(self, now):
        if self.volume is None:
            return
        if self.warning.active and self.audio_enabled:
            if not self.low_volume_asserted and now >= self.next_volume_retry:
                self.low_volume_asserted = self.maximize_volume(now)
                self.next_volume_retry = now + VOLUME_RETRY
        elif self.child_kind != "startup":
            self.low_volume_asserted = False
            if (self.volume.saved is not None or self.volume.error) and now >= self.next_volume_retry:
                try:
                    self.volume.restore()
                except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
                    self.warn("Cannot restore prompt volume: {}".format(error), now)
                    self.next_volume_retry = now + VOLUME_RETRY

    def launch(self, kind, now):
        sound = self.sound_dir / ("startup.wav" if kind == "startup" else "low_battery.wav")
        try:
            descriptor = acquire_speaker(OUTPUT_LOCK)
        except OSError as error:
            self.warn("Cannot check speaker availability: {}".format(error), now)
            descriptor = None
        if descriptor is None:
            self.warn("Speaker is in use; prompt will retry", now)
            if kind == "startup":
                self.next_startup = now + BUSY_RETRY
            else:
                self.next_warning = now + BUSY_RETRY
            return
        if kind == "low_battery":
            # Preserve the spacing if this service is restarted during playback.
            self.last_warning_attempt = now
            self.sync_timing_marker(now)
        try:
            # The lock remains held by the player until it finishes, so no
            # other sound can start between changing the mixer and playback.
            self.maximize_volume(now)
            self.child = subprocess.Popen(
                [sys.executable, str(self.player), "--lock-fd", str(descriptor),
                 "--warmup-prompt-ms", str(PROMPT_WARMUP_MS), "-q", "-D", self.device, str(sound)],
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                pass_fds=(descriptor,),
            )
        except OSError as error:
            self.warn("Cannot start prompt playback: {}".format(error), now)
            if kind == "startup":
                self.next_startup = now + PROMPT_INTERVAL
            else:
                self.next_warning = now + PROMPT_INTERVAL
            return
        finally:
            os.close(descriptor)
        self.child_kind, self.child_started = kind, now

    def finish_child(self, now):
        if self.child is None:
            return
        result = self.child.poll()
        if result is None and now - self.child_started >= PLAYER_TIMEOUT:
            self.child.terminate()
            try:
                self.child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait()
            result = -1
        if result is None:
            return
        kind, started = self.child_kind, self.child_started
        self.child, self.child_kind = None, None
        if result == EX_TEMPFAIL:
            self.warn("Speaker is in use; prompt will retry", now)
            if kind == "startup":
                self.next_startup = now + BUSY_RETRY
            else:
                self.next_warning = now + BUSY_RETRY
        elif result == 0:
            if kind == "startup":
                self.startup_pending = False
            else:
                # Successful, uncontended starts are spaced by five seconds.
                self.next_warning = started + PROMPT_INTERVAL
        else:
            self.warn("Prompt playback failed; retrying later", now)
            if kind == "startup":
                self.next_startup = now + PROMPT_INTERVAL
            else:
                self.next_warning = now + PROMPT_INTERVAL

    def tick(self, now):
        self.finish_child(now)
        if now >= self.next_poll:
            sample = read_voltage(self.socket_path)
            if sample is None or sample[0] is None:
                fallback = self.maintenance_voltage.observe(self.maintenance_reader(), time.monotonic())
                if fallback is not None:
                    sample = fallback
            else:
                self.maintenance_voltage.reset()
            if self.warning.observe(sample, now):
                saved = self.sync_warning_marker(now)
                if self.warning.active:
                    self.low_volume_asserted = False
                    self.next_volume_retry = 0.0
                    self.next_warning = now
                elif saved:
                    self.last_warning_attempt = None
                    self.next_volume_retry = 0.0
                else:
                    # A failed removal cannot prove the warning will stay cleared
                    # after a restart. Retry once another fresh reading arrives.
                    self.warning.active = True
            elif self.warning.active and not self.warning_saved:
                self.sync_warning_marker(now)
            self.next_poll = time.monotonic() + POLL_INTERVAL
        self.sync_volume(now)
        if self.child is not None or not self.audio_enabled:
            return
        if self.startup_pending:
            if now >= self.startup_deadline:
                self.startup_pending = False
            elif now >= self.next_startup:
                self.launch("startup", now)
                return
        if self.warning.active and now >= self.next_warning:
            self.launch("low_battery", now)

    def run(self):
        while not self.stopping:
            self.tick(time.monotonic())
            time.sleep(0.1)
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:
                self.child.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait()
        self.child, self.child_kind = None, None
        if not self.warning.active:
            self.next_volume_retry = 0.0
            self.sync_volume(time.monotonic())


def main():
    service = PromptService()
    def stop(_signum, _frame):
        service.stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    service.run()


if __name__ == "__main__":
    main()
