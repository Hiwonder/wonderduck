#!/usr/bin/python3
"""Run aplay while holding the board amplifier enable; accepts unchanged aplay arguments."""

import fcntl
import os
import signal
import struct
import subprocess
import sys

from audio_speaker import speaker_enabled

OUTPUT_LOCK = "/run/lock/audio-output.lock"
WARMUP_MS = 500
COPY_BLOCK = 65536


def argument_value(arguments, short, long):
    for index, argument in enumerate(arguments):
        if argument in (short, long):
            return arguments[index + 1] if index + 1 < len(arguments) else None
        if argument.startswith(long + "="):
            return argument[len(long) + 1:]
    return None


def wav_warmup(source, duration_ms=WARMUP_MS):
    """Prepare an extended WAV header while leaving its audio data in the source."""
    if not source.seekable():
        return None
    start = source.tell()
    prepared = False
    try:
        header = source.read(12)
        if len(header) != 12 or header[:4] != b"RIFF" or header[8:] != b"WAVE":
            return None
        riff_size = struct.unpack_from("<I", header, 4)[0]
        prefix = bytearray(header)
        format_info = None
        while len(prefix) <= 4 * 1024 * 1024:
            chunk = source.read(8)
            if len(chunk) != 8:
                return None
            name, length = struct.unpack("<4sI", chunk)
            if name == b"data":
                if format_info is None:
                    return None
                channels, rate, block_align, bits, format_tag = format_info
                added = (rate * duration_ms // 1000) * block_align
                if not added or length % block_align:
                    return None
                new_length = length + added
                remaining = riff_size + 8 - (source.tell() - start)
                if remaining < length:
                    return None
                # Some WAV writers omit the last pad byte when data is the final chunk.
                old_padding = (length & 1) if remaining > length else 0
                new_padding = new_length & 1
                adjustment = new_padding - old_padding
                if new_length > 0xFFFFFFFF or riff_size + added + adjustment > 0xFFFFFFFF:
                    return None
                struct.pack_into("<I", prefix, 4, riff_size + added + adjustment)
                prefix.extend(struct.pack("<4sI", name, new_length))
                silence_byte = b"\x80" if format_tag == 1 and bits == 8 else b"\x00"
                prepared = True
                return bytes(prefix), added, silence_byte, length, old_padding, new_padding
            padded = length + (length & 1)
            if len(prefix) + 8 + padded > 4 * 1024 * 1024:
                return None
            data = source.read(padded)
            if len(data) != padded:
                return None
            prefix.extend(chunk)
            prefix.extend(data)
            if name == b"fmt ":
                if length < 16:
                    return None
                format_tag, channels, rate, _, block_align, bits = struct.unpack_from("<HHIIHH", data)
                if format_tag == 0xFFFE and length >= 40:
                    guid = data[24:40]
                    if guid[4:] != bytes.fromhex("00001000800000aa00389b71"):
                        return None
                    format_tag = struct.unpack_from("<I", guid)[0]
                if (format_tag not in (1, 3) or not 1 <= channels <= 16 or not 1 <= rate <= 384000
                        or bits not in (8, 16, 24, 32, 64)
                        or not channels * ((bits + 7) // 8) <= block_align <= channels * 8
                        or (format_tag == 3 and bits not in (32, 64))):
                    return None
                format_info = channels, rate, block_align, bits, format_tag
    finally:
        # A successful parse leaves the cursor at the original data chunk.
        if not prepared:
            source.seek(start)
    return None


def prepare_warmup(arguments, duration_ms=WARMUP_MS):
    """Warm supported finite playback; leave tightly paced real-time streams alone."""
    if not 0 <= duration_ms <= 2000:
        raise ValueError("音频预热时长无效")
    if duration_ms == 0:
        return None
    if any(item.startswith(("--buffer-time", "--period-time")) for item in arguments):
        return None
    kind = argument_value(arguments, "-t", "--file-type")
    if kind == "raw":
        sample_format = argument_value(arguments, "-f", "--format")
        formats = {"S8": (1, b"\x00"), "U8": (1, b"\x80"), "S16_LE": (2, b"\x00"),
                   "S32_LE": (4, b"\x00"), "FLOAT_LE": (4, b"\x00")}
        try:
            width, silence = formats[sample_format]
            channels = int(argument_value(arguments, "-c", "--channels"))
            rate = int(argument_value(arguments, "-r", "--rate"))
        except (KeyError, TypeError, ValueError):
            return None
        if not 1 <= channels <= 16 or not 1 <= rate <= 384000:
            return None
        source, opened, output_arguments = sys.stdin.buffer, False, arguments
        if (arguments and os.path.isfile(arguments[-1]) and len(arguments) > 1
                and arguments[-2] not in ("-D", "--device", "-t", "--file-type", "-f", "--format",
                                          "-c", "--channels", "-r", "--rate")):
            source, opened, output_arguments = open(arguments[-1], "rb"), True, arguments[:-1]
        return output_arguments, source, opened, ("raw", rate * duration_ms // 1000 * channels * width, silence)
    opened = False
    if arguments and arguments[-1].lower().endswith(".wav") and os.path.isfile(arguments[-1]):
        source = open(arguments[-1], "rb")
        opened = True
        output_arguments = arguments[:-1]
    elif kind == "wav":
        source = sys.stdin.buffer
        output_arguments = arguments
    else:
        return None
    try:
        plan = wav_warmup(source, duration_ms)
        if plan is None:
            if opened:
                source.close()
            return None
        return output_arguments, source, opened, ("wav", *plan)
    except BaseException:
        if opened:
            source.close()
        raise


def copy_stream(source, destination, count=None):
    while count is None or count > 0:
        size = COPY_BLOCK if count is None else min(COPY_BLOCK, count)
        reader = getattr(source, "read1", source.read)
        chunk = reader(size)
        if not chunk:
            if count is not None and count:
                raise ValueError("音频数据不完整")
            return
        destination.write(chunk)
        if count is not None:
            count -= len(chunk)


def feed_warmup(source, destination, plan):
    kind = plan[0]
    if kind == "wav":
        _, header, added, silence, original_size, old_padding, new_padding = plan
        destination.write(header)
    else:
        _, added, silence = plan
    block = silence * min(COPY_BLOCK, added)
    while added:
        size = min(len(block), added)
        destination.write(block[:size])
        added -= size
    if kind == "wav":
        copy_stream(source, destination, original_size)
        if old_padding and len(source.read(old_padding)) != old_padding:
            raise ValueError("音频数据不完整")
        if new_padding:
            destination.write(b"\x00")
    copy_stream(source, destination)


def terminate(signum, _frame):
    raise SystemExit(128 + signum)


def play(arguments, if_free=False):
    # A superseded player is still cleaning up its GPIO. Wait here, outside the control
    # process, so a replacement cannot claim the pin before the old player drives it low.
    try:
        fd = os.open(OUTPUT_LOCK, os.O_RDONLY)
    except FileNotFoundError:
        try:
            fd = os.open(OUTPUT_LOCK, os.O_RDONLY | os.O_CREAT | os.O_EXCL, 0o666)
        except FileExistsError:
            fd = os.open(OUTPUT_LOCK, os.O_RDONLY)
    with os.fdopen(fd, "r") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | (fcntl.LOCK_NB if if_free else 0))
        except BlockingIOError:
            # A prompt may retry later without interrupting a recording or another sound.
            return 75
        return play_enabled(arguments)


def play_with_lock(arguments, fd):
    # Prompt playback has already acquired this same open file description.
    with os.fdopen(fd, "r") as lock:
        held, expected = os.fstat(lock.fileno()), os.stat(OUTPUT_LOCK)
        if (held.st_dev, held.st_ino) != (expected.st_dev, expected.st_ino):
            raise RuntimeError("音频锁无效")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return play_enabled(arguments)


def play_enabled(arguments):
    duration_ms = WARMUP_MS
    prompt = False
    if arguments and arguments[0] == "--warmup-prompt-ms":
        if len(arguments) < 3:
            raise ValueError("缺少提示音预热时长或文件")
        duration_ms, prompt = int(arguments[1]), True
        arguments = arguments[2:]
    stream = prepare_warmup(arguments, duration_ms)
    if prompt and duration_ms and stream is None:
        raise ValueError("提示音格式不支持预热")
    if stream is not None:
        arguments = stream[0]
    try:
        with speaker_enabled(verbose=False, device=argument_value(arguments, "-D", "--device")):
            process = subprocess.Popen(["aplay", *arguments],
                                       stdin=subprocess.PIPE if stream is not None else sys.stdin)
            try:
                if stream is not None:
                    try:
                        feed_warmup(stream[1], process.stdin, stream[3])
                    except BrokenPipeError:
                        pass
                    finally:
                        try:
                            process.stdin.close()
                        except BrokenPipeError:
                            pass
                return process.wait()
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
    finally:
        if stream is not None and stream[2]:
            stream[1].close()


def main():
    signal.signal(signal.SIGTERM, terminate)
    try:
        arguments = sys.argv[1:]
        if arguments and arguments[0] == "--lock-fd":
            if len(arguments) < 2:
                raise ValueError("缺少音频锁描述符")
            return play_with_lock(arguments[2:], int(arguments[1]))
        if_free = bool(arguments and arguments[0] == "--if-free")
        return play(arguments[1:] if if_free else arguments, if_free=if_free)
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print("音频播放失败：{}".format(error), file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
