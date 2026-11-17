#!/usr/bin/env python3
"""Prepare and confirm recovery from a quarantined status UART transaction."""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import tempfile


JOURNAL = Path('/var/lib/robotd-status-read.pending')
PREPARATION = Path('/var/lib/robotd-status-read.recovery')
INSTANCE_LOCK = Path('/run/robotd.sock.lock')
BOOT_ID = Path('/proc/sys/kernel/random/boot_id')
MAX_JOURNAL_BYTES = 4096
UUID_PATTERN = re.compile(r'[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\Z')


class RecoveryError(Exception):
    pass


def current_boot_id(path=BOOT_ID):
    value = path.read_text(encoding='ascii').strip().lower()
    if not UUID_PATTERN.fullmatch(value):
        raise RecoveryError('无法确认系统启动标识')
    return value


def ensure_stopped():
    """Check both systemd and standalone robotd instances; never stop either."""
    result = subprocess.run(['systemctl', 'show', 'robotd.service', '--property=ActiveState',
                             '--property=MainPID'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=5)
    values = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    if result.returncode or values.get('ActiveState') not in ('inactive', 'failed') or values.get('MainPID') != '0':
        raise RecoveryError('robotd 服务尚未停止或无法确认状态，请先手动停止')
    result = subprocess.run(['pgrep', '-x', 'robotd'], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, timeout=5)
    if result.returncode != 1:
        raise RecoveryError('robotd 进程仍在运行或无法确认，请先停止所有实例')


def regular_metadata(path, owner):
    """Reject unsafe objects before attempting to open them."""
    info = path.lstat()
    if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != owner
            or info.st_size > MAX_JOURNAL_BYTES):
        raise RecoveryError('状态文件类型、所有者或大小异常，未作更改')
    return info


def file_signature(info):
    return (info.st_dev, info.st_ino, info.st_mode, info.st_uid, info.st_nlink,
            info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def checked_regular(path, owner):
    """Read only a stable, owned regular file without following links."""
    before = regular_metadata(path, owner)
    # NONBLOCK prevents a substituted FIFO from hanging the recovery process.
    descriptor = os.open(str(path), os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if file_signature(before) != file_signature(info):
            raise RecoveryError('状态文件在打开时发生变化，未作更改')
        with os.fdopen(descriptor, 'rb', closefd=False) as stream:
            data = stream.read(MAX_JOURNAL_BYTES + 1)
        if len(data) != info.st_size or len(data) > MAX_JOURNAL_BYTES:
            raise RecoveryError('状态文件在读取时发生变化，未作更改')
        after = os.fstat(descriptor)
        current = regular_metadata(path, owner)
        if file_signature(info) != file_signature(after) or file_signature(info) != file_signature(current):
            raise RecoveryError('状态文件在读取时发生变化，未作更改')
        return data, dict(dev=info.st_dev, ino=info.st_ino, size=info.st_size,
                          mtime_ns=info.st_mtime_ns, ctime_ns=info.st_ctime_ns,
                          sha256=hashlib.sha256(data).hexdigest())
    finally:
        os.close(descriptor)


def journal_boot_id(data):
    """Recognise complete new records; unknown/old/partial records use preparation only."""
    try:
        if not data.endswith(b'\n'):
            return None
        lines = data.decode('ascii').splitlines()
        fields = dict(line.split('=', 1) for line in lines)
        if len(fields) != len(lines) or fields.get('version') != '1':
            return None
        value = fields.get('boot_id', '')
        if not UUID_PATTERN.fullmatch(value):
            return None
        if fields.get('kind') == 'session' and set(fields) == {'version', 'boot_id', 'kind'}:
            return value
        if fields.get('kind') != 'read' or set(fields) != {'version', 'boot_id', 'kind', 'id', 'address', 'length'}:
            return None
        for key in ('id', 'address', 'length'):
            if not fields[key].isascii() or not fields[key].isdigit():
                return None
        device, address, length = (int(fields[key]) for key in ('id', 'address', 'length'))
        if not 0 <= device <= 254 or not 0 <= address <= 255 or not 1 <= length <= 255 or address + length > 256:
            return None
        return value
    except (ValueError, UnicodeError, OverflowError):
        return None


def sync_directory(path):
    descriptor = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


@contextlib.contextmanager
def instance_lock(path, owner):
    # The inode is shared with robotd; never unlink or replace it.
    try:
        before = regular_metadata(path, owner)
    except FileNotFoundError:
        before = None
    descriptor = os.open(str(path), os.O_RDWR | os.O_CREAT | os.O_NONBLOCK | os.O_NOFOLLOW | os.O_CLOEXEC, 0o644)
    try:
        info = os.fstat(descriptor)
        if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != owner
                or (before is not None and (before.st_dev, before.st_ino) != (info.st_dev, info.st_ino))):
            raise RecoveryError('robotd 实例锁文件异常，未作更改')
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RecoveryError('robotd 或其他操作正在占用实例锁') from error
        ensure_lock_unchanged(path, descriptor)
        yield descriptor
    finally:
        os.close(descriptor)


def ensure_lock_unchanged(path, descriptor):
    opened = os.fstat(descriptor)
    current = os.stat(str(path), follow_symlinks=False)
    if (not stat.S_ISREG(opened.st_mode) or not stat.S_ISREG(current.st_mode)
            or opened.st_nlink != 1 or current.st_nlink != 1 or opened.st_uid != current.st_uid
            or (opened.st_dev, opened.st_ino) != (current.st_dev, current.st_ino)):
        raise RecoveryError('robotd 实例锁已变化，未作更改')


def write_preparation(path, boot, fingerprint, owner):
    try:
        _, existing = checked_regular(path, owner)
    except FileNotFoundError:
        existing = None
    descriptor, temporary = tempfile.mkstemp(prefix='.robotd-status-recovery-', dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='ascii') as stream:
            os.fchmod(stream.fileno(), 0o600)
            json.dump(dict(schema=1, boot_id=boot, journal=fingerprint), stream, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        if existing is not None:
            if checked_regular(path, owner)[1] != existing:
                raise RecoveryError('恢复准备文件已变化，未作更改')
        else:
            try:
                path.lstat()
            except FileNotFoundError:
                pass
            else:
                raise RecoveryError('恢复准备文件已出现，未作更改')
        os.replace(temporary, str(path))
        sync_directory(path.parent)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_preparation(path, owner):
    raw, fingerprint = checked_regular(path, owner)
    try:
        value = json.loads(raw)
        if (not isinstance(value, dict) or set(value) != {'schema', 'boot_id', 'journal'}
                or value['schema'] != 1 or not isinstance(value['boot_id'], str)
                or not UUID_PATTERN.fullmatch(value['boot_id']) or not isinstance(value['journal'], dict)):
            raise ValueError('invalid preparation')
        return value, fingerprint
    except (ValueError, TypeError, UnicodeError) as error:
        raise RecoveryError('恢复准备文件无效，请重新准备并完全断电') from error


def prepare(journal=JOURNAL, preparation=PREPARATION, lock_path=INSTANCE_LOCK,
            boot_file=BOOT_ID, owner=0, check_stopped=ensure_stopped):
    check_stopped()
    with instance_lock(lock_path, owner) as lock:
        check_stopped()
        ensure_lock_unchanged(lock_path, lock)
        boot = current_boot_id(boot_file)
        _, fingerprint = checked_regular(journal, owner)
        write_preparation(preparation, boot, fingerprint, owner)


def confirm_power_cycle():
    if not sys.stdin.isatty():
        raise RecoveryError('必须在终端人工确认，不能通过管道自动确认')
    print('请确认：机器人、全部舵机和 MCU 已完全断开电池、充电器、USB 及所有其他电源，')
    print('随后重新接通并开机；单独重启主板不满足要求。')
    if input('确认请完整输入 POWER-CYCLED：').strip() != 'POWER-CYCLED':
        raise RecoveryError('未确认完全断电，未作更改')


def recover(journal=JOURNAL, preparation=PREPARATION, lock_path=INSTANCE_LOCK,
            boot_file=BOOT_ID, owner=0, check_stopped=ensure_stopped, confirm=confirm_power_cycle):
    check_stopped()
    with instance_lock(lock_path, owner) as lock:
        check_stopped()
        ensure_lock_unchanged(lock_path, lock)
        boot = current_boot_id(boot_file)
        data, fingerprint = checked_regular(journal, owner)
        prepared, preparation_fingerprint = load_preparation(preparation, owner)
        if prepared['journal'] != fingerprint:
            raise RecoveryError('状态文件与准备时不一致，请重新准备并完全断电')
        if prepared['boot_id'] == boot:
            raise RecoveryError('准备后系统尚未重新开机，请完全断电后再试')
        recorded = journal_boot_id(data)
        if recorded is not None and recorded == boot:
            raise RecoveryError('状态故障发生于本次开机，请重新准备并完全断电')
        confirm()
        check_stopped()
        ensure_lock_unchanged(lock_path, lock)
        if (current_boot_id(boot_file) != boot or checked_regular(journal, owner)[1] != fingerprint
                or checked_regular(preparation, owner)[1] != preparation_fingerprint):
            raise RecoveryError('确认期间状态发生变化，未作更改')
        # Removing preparation first is fail-closed: an interruption requires a
        # new preparation and another full power cycle before trying again.
        os.unlink(str(preparation))
        sync_directory(preparation.parent)
        os.unlink(str(journal))
        try:
            sync_directory(journal.parent)
        except OSError as error:
            # Do not report success if durable removal cannot be confirmed.
            try:
                descriptor = os.open(str(journal), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
                with os.fdopen(descriptor, 'wb') as stream:
                    stream.write(data)
                    stream.flush()
                    os.fsync(stream.fileno())
                sync_directory(journal.parent)
            except OSError:
                pass
            raise RecoveryError('状态文件持久删除未确认；保持服务停止并检查存储') from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'recover'))
    args = parser.parse_args()
    try:
        if os.geteuid() != 0:
            raise RecoveryError('请使用 sudo 运行')
        if args.action == 'prepare':
            prepare()
            print('已准备。请关闭设备，完全断开所有电源后重新开机，再执行 recover。')
        else:
            recover()
            print('状态串口故障标记已清除。请按现有方式启动 robotd。')
    except (RecoveryError, OSError, subprocess.SubprocessError, KeyboardInterrupt, EOFError) as error:
        print('恢复未完成：{}'.format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
