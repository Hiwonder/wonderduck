#!/usr/bin/env python3
"""Back up an offline GPT/ext4 system card to a smaller bootable image."""

import argparse
import datetime
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import time

from tf_image_sanitize import sanitize


SECTOR = 512
MIB = 1024 * 1024
UNIT_NAME = "wonderduck-expand-root.service"
EXPANDER = "/usr/local/sbin/wonderduck-expand-root"
MARKER = "/var/lib/wonderduck/root-expanded"
LEGACY_EXPANDER_SHA256 = "4f3e66b8639954fd1c9ef00ab15c00b6e3a675ea064b623f6c60f48aa8c18900"
BLUETOOTH_UNIT_NAME = "wonderduck-reset-bluetooth.service"
BLUETOOTH_RESET = "/usr/local/sbin/wonderduck-reset-bluetooth"
BLUETOOTH_MARKER = "/var/lib/wonderduck/bluetooth-reset-pending"
SSH_UNIT_NAME = "wonderduck-generate-ssh-host-keys.service"
SSH_MARKER = "/var/lib/wonderduck/ssh-keys-pending"
UNIT = """[Unit]
Description=Expand root filesystem on first boot
After=local-fs.target resize-all.service
Before=multi-user.target
ConditionPathExists=!/var/lib/wonderduck/root-expanded

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/wonderduck-expand-root
TimeoutStartSec=15min

[Install]
WantedBy=multi-user.target
"""
BLUETOOTH_UNIT = """[Unit]
Description=Remove copied Bluetooth pairings on first boot
After=local-fs.target
Before=bluetooth.service
ConditionPathExists=/var/lib/wonderduck/bluetooth-reset-pending

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/wonderduck-reset-bluetooth

[Install]
RequiredBy=bluetooth.service
"""
SSH_UNIT = """[Unit]
Description=Generate unique SSH host keys on first boot
After=local-fs.target systemd-random-seed.service
Before=ssh.service sshd.service
ConditionPathExists=/var/lib/wonderduck/ssh-keys-pending

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/wonderduck-generate-ssh-host-keys

[Install]
RequiredBy=ssh.service sshd.service
"""
SSH_GENERATOR = """#!/bin/sh
set -eu
/usr/bin/ssh-keygen -A
test -s /etc/ssh/ssh_host_ed25519_key || test -s /etc/ssh/ssh_host_ecdsa_key || test -s /etc/ssh/ssh_host_rsa_key
rm -f /var/lib/wonderduck/ssh-keys-pending
"""


def run(args, *, capture=False, input_text=None, allowed=(0,)):
    result = subprocess.run([str(arg) for arg in args], input=input_text, text=True,
                            stdout=subprocess.PIPE if capture else None,
                            stderr=subprocess.PIPE if capture else None,
                            env=dict(os.environ, LC_ALL="C"))
    if result.returncode not in allowed:
        detail = (result.stderr or result.stdout or "").strip() if capture else ""
        raise RuntimeError("命令执行失败：{}{}".format(" ".join(map(str, args)),
                                                  "\n" + detail if detail else ""))
    return result


def find_source():
    data = json.loads(run(["lsblk", "--json", "--bytes", "--nodeps", "-o",
                           "PATH,TYPE,RM,SIZE,MODEL"], capture=True).stdout)
    choices = [item for item in data["blockdevices"]
               if item["type"] == "disk" and item["rm"] and int(item["size"] or 0) > 0]
    if not choices:
        raise RuntimeError("没有发现可移动 TF 卡；请插卡后重试，或指定整盘设备路径")
    if len(choices) == 1:
        return Path(choices[0]["path"])
    print("发现多个可移动磁盘：")
    for number, item in enumerate(choices, 1):
        print("  {}. {}  {:.1f} GiB  {}".format(number, item["path"], int(item["size"]) / 2**30,
                                               item.get("model") or ""))
    if not sys.stdin.isatty():
        raise RuntimeError("请指定需要备份的整盘设备路径")
    choice = input("输入要备份的磁盘编号：").strip()
    if not choice.isdigit() or not 1 <= int(choice) <= len(choices):
        raise RuntimeError("磁盘编号无效")
    return Path(choices[int(choice) - 1]["path"])


def prepare_source(source, output_dir):
    source = source.resolve(strict=True)
    mode = source.stat().st_mode
    if stat.S_ISREG(mode):
        return source, False
    if not stat.S_ISBLK(mode):
        raise RuntimeError("备份源必须是整盘设备或已有的磁盘镜像")
    if os.geteuid() != 0:
        print("读取 TF 卡需要管理员权限，正在请求 sudo。", flush=True)
        os.execvp("sudo", ["sudo", "--", sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]])
    info = json.loads(run(["lsblk", "--json", "--bytes", "--tree", "-o", "PATH,TYPE,LOG-SEC", source],
                          capture=True).stdout)["blockdevices"]
    if len(info) != 1 or info[0]["type"] != "disk" or info[0]["log-sec"] != SECTOR:
        raise RuntimeError("需要 512 字节扇区的整盘设备，不能选择分区或系统虚拟盘")
    nodes = []
    def collect(node):
        nodes.append(node)
        for child in node.get("children", []):
            collect(child)
    collect(info[0])
    mounts = []
    for node in nodes:
        result = run(["findmnt", "--list", "--json", "--source", node["path"], "--output", "TARGET"],
                     capture=True, allowed=(0, 1))
        if result.returncode == 0:
            mounts.extend((node["path"], item["target"]) for item in json.loads(result.stdout)["filesystems"])
    if any(mount in ("/", "/boot", "/boot/efi") for _, mount in mounts):
        raise RuntimeError("所选磁盘包含本机系统挂载点，拒绝操作")
    if any(os.path.commonpath((str(output_dir), mount)) == mount for _, mount in mounts):
        raise RuntimeError("输出目录位于源磁盘上，不能在卸载源卡后写入")
    swaps = Path("/proc/swaps").read_text().splitlines()[1:]
    if any(line.split()[0] in {node["path"] for node in nodes} for line in swaps):
        raise RuntimeError("所选磁盘正在被本机用于交换空间，拒绝操作")
    for device, mount in sorted(mounts, key=lambda entry: len(entry[1]), reverse=True):
        print("卸载 {}：{}".format(device, mount), flush=True)
        run(["umount", mount])
    return source, True


def partition_table(source, block_device):
    table = json.loads(run(["sfdisk", "--json", source], capture=True).stdout)["partitiontable"]
    if table.get("label") != "gpt" or table.get("sectorsize", SECTOR) != SECTOR:
        raise RuntimeError("只支持 512 字节扇区的 GPT 系统卡")
    partitions = table.get("partitions", [])
    roots = [part for part in partitions if part.get("name", "").lower() == "rootfs"]
    if len(roots) != 1:
        raise RuntimeError("未找到唯一的 rootfs GPT 分区")
    root = roots[0]
    if any(part["start"] > root["start"] for part in partitions):
        raise RuntimeError("rootfs 不是最后一个分区，不能安全缩小镜像")
    if any(part["start"] + part["size"] > root["start"] for part in partitions if part is not root):
        raise RuntimeError("源磁盘分区发生重叠")
    probe = ["blkid", "-p", "-s", "TYPE", "-o", "value"]
    probe += [root["node"]] if block_device else ["-O", str(root["start"] * SECTOR), source]
    if run(probe, capture=True).stdout.strip() != "ext4":
        raise RuntimeError("rootfs 不是 ext4 文件系统")
    return table, root


def debugfs(image, command, write=False):
    result = run(["debugfs", *( ["-w"] if write else [] ), "-R", command, image], capture=True)
    errors = result.stderr.splitlines()[1:]
    return result.stdout, "\n".join(errors).strip()


def debugfs_arg(value):
    value = str(value)
    if any(character in value for character in ('"', "\\", "\n", "\r", "\x00")):
        raise RuntimeError("镜像路径包含无法安全处理的字符")
    return '"' + value + '"'


def inspect_path(image, path):
    output, error = debugfs(image, "stat " + debugfs_arg(path))
    if "File not found" in error:
        return None
    if error or not output.startswith("Inode:"):
        raise RuntimeError("无法检查镜像内的 {}：{}".format(path, error or output.strip()))
    return output


def modify(image, command):
    _, error = debugfs(image, command, write=True)
    if error:
        raise RuntimeError("修改镜像失败：{}：{}".format(command, error))


def write_file(image, source, destination):
    parent, name = destination.rsplit("/", 1)
    result = run(["debugfs", "-w", "-f", "-", image],
                 input_text="cd {}\nwrite {} {}\n".format(debugfs_arg(parent), debugfs_arg(source),
                                                            debugfs_arg(name)), capture=True)
    errors = "\n".join(result.stderr.splitlines()[1:]).strip()
    if errors or not inspect_path(image, destination):
        raise RuntimeError("写入镜像失败：{}：{}".format(destination, errors))


def install_expand_once(image, tempdir):
    directories = ("/usr/local/sbin", "/usr/local/bin", "/usr/sbin", "/usr/bin", "/sbin", "/bin")
    for command in ("findmnt", "readlink", "lsblk", "blkid", "sgdisk", "parted", "partx", "blockdev", "resize2fs"):
        if not any(inspect_path(image, directory + "/" + command) for directory in directories):
            raise RuntimeError("镜像中缺少首次扩容所需工具：" + command)
    script = inspect_path(image, EXPANDER)
    unit_path = "/etc/systemd/system/" + UNIT_NAME
    alternative = "/lib/systemd/system/" + UNIT_NAME
    unit = inspect_path(image, unit_path)
    if not unit and inspect_path(image, alternative):
        unit_path, unit = alternative, True
    if not script:
        candidates = [Path(__file__).resolve().with_name("expand-rootfs"),
                      Path(__file__).resolve().parent.parent / "deploy/system/expand-rootfs"]
        source = next((path for path in candidates if path.is_file()), None)
        if source is None:
            raise RuntimeError("镜像没有扩容脚本，且找不到配套的 expand-rootfs 文件")
        staged = tempdir / "expand-rootfs"
        shutil.copyfile(source, staged)
        write_file(image, staged, EXPANDER)
        modify(image, "set_inode_field {} mode 0100755".format(EXPANDER))
        print("镜像内已安装配套扩容脚本。", flush=True)
    else:
        content, error = debugfs(image, "cat " + EXPANDER)
        if error:
            raise RuntimeError("无法读取镜像内的扩容脚本：" + error)
        if hashlib.sha256(content.encode("utf-8")).hexdigest() == LEGACY_EXPANDER_SHA256:
            old = 'sgdisk -e "$disk"\n'
            if content.count(old) != 1:
                raise RuntimeError("旧扩容脚本内容不符合预期")
            fixed = content.replace(old, "printf 'Fix\\n' | parted ---pretend-input-tty \"$disk\" print >/dev/null\n", 1)
            staged = tempdir / "expand-rootfs-fixed"
            staged.write_text(fixed, encoding="utf-8")
            modify(image, "rm " + EXPANDER)
            write_file(image, staged, EXPANDER)
            modify(image, "set_inode_field {} mode 0100755".format(EXPANDER))
            print("已修复镜像内扩容脚本的 GPT 交互问题。", flush=True)
        else:
            print("镜像内已有扩容脚本，继续使用原文件。", flush=True)
    if not unit:
        staged = tempdir / UNIT_NAME
        staged.write_text(UNIT, encoding="utf-8")
        write_file(image, staged, unit_path)
        modify(image, "set_inode_field {} mode 0100644".format(unit_path))
    content, error = debugfs(image, "cat " + unit_path)
    if (error or not re.search(r"(?m)^\s*ExecStart=" + re.escape(EXPANDER) + r"\s*$", content)
            or not re.search(r"(?m)^\s*ConditionPathExists=!" + re.escape(MARKER) + r"\s*$", content)
            or not re.search(r"(?m)^\s*WantedBy=multi-user\.target\s*$", content)):
        raise RuntimeError("镜像内扩容服务与一次性扩容要求不一致，请先检查：" + unit_path)
    wants = "/etc/systemd/system/multi-user.target.wants"
    if not inspect_path(image, wants):
        modify(image, "mkdir " + wants)
    link = wants + "/" + UNIT_NAME
    existing = inspect_path(image, link)
    if existing:
        target = re.search(r'Fast link dest: "([^"]+)"', existing)
        if not target or target.group(1) not in ("../" + UNIT_NAME, unit_path):
            raise RuntimeError("镜像内的扩容服务启用链接指向其他位置：" + link)
    else:
        result = run(["debugfs", "-w", "-f", "-", image],
                     input_text="cd {}\nsymlink {} {}\n".format(wants, UNIT_NAME, unit_path), capture=True)
        errors = "\n".join(result.stderr.splitlines()[1:]).strip()
        if errors:
            raise RuntimeError("启用扩容服务失败：" + errors)
    if inspect_path(image, MARKER):
        modify(image, "rm " + MARKER)
    if inspect_path(image, MARKER) or not inspect_path(image, link):
        raise RuntimeError("无法确认镜像中的首次扩容设置")


def install_bluetooth_reset_once(image, tempdir):
    # Nothing can have been copied from BlueZ when the state directory does
    # not exist in the source filesystem.
    if not inspect_path(image, "/var/lib/bluetooth"):
        return
    candidates = [Path(__file__).resolve().with_name("reset-bluetooth-pairings"),
                  Path(__file__).resolve().parent.parent / "deploy/system/reset-bluetooth-pairings"]
    source = next((path for path in candidates if path.is_file()), None)
    if source is None:
        raise RuntimeError("找不到配套的 reset-bluetooth-pairings 文件")
    if not inspect_path(image, "/usr/bin/python3"):
        raise RuntimeError("镜像中缺少首次重置蓝牙所需的 Python 3")
    for directory in ("/var/lib/wonderduck", "/etc/systemd/system/bluetooth.service.requires"):
        if not inspect_path(image, directory):
            modify(image, "mkdir " + directory)
    unit_path = "/etc/systemd/system/" + BLUETOOTH_UNIT_NAME
    script = tempdir / "reset-bluetooth-pairings"
    shutil.copyfile(source, script)
    staged = tempdir / BLUETOOTH_UNIT_NAME
    staged.write_text(BLUETOOTH_UNIT, encoding="utf-8")
    marker = tempdir / "bluetooth-reset-pending"
    marker.write_text("", encoding="utf-8")
    for path, content, mode in ((BLUETOOTH_RESET, script, "0100755"),
                                (unit_path, staged, "0100644"),
                                (BLUETOOTH_MARKER, marker, "0100644")):
        if inspect_path(image, path):
            modify(image, "rm " + path)
        write_file(image, content, path)
        modify(image, "set_inode_field {} mode {}".format(path, mode))
    requires = "/etc/systemd/system/bluetooth.service.requires"
    link = requires + "/" + BLUETOOTH_UNIT_NAME
    existing = inspect_path(image, link)
    if existing:
        target = re.search(r'Fast link dest: "([^"]+)"', existing)
        if not target or target.group(1) not in ("../" + BLUETOOTH_UNIT_NAME, unit_path):
            raise RuntimeError("镜像内的蓝牙重置服务链接指向其他位置：" + link)
    else:
        result = run(["debugfs", "-w", "-f", "-", image],
                     input_text="cd {}\nsymlink {} {}\n".format(requires, BLUETOOTH_UNIT_NAME, unit_path),
                     capture=True)
        errors = "\n".join(result.stderr.splitlines()[1:]).strip()
        if errors:
            raise RuntimeError("启用首次蓝牙重置失败：" + errors)
    if not inspect_path(image, link) or not inspect_path(image, BLUETOOTH_MARKER):
        raise RuntimeError("无法确认镜像中的首次蓝牙重置设置")


def install_ssh_key_generation(image, tempdir):
    if not inspect_path(image, "/etc/ssh/sshd_config"):
        return
    if not inspect_path(image, "/usr/bin/ssh-keygen"):
        raise RuntimeError("镜像中缺少首次生成 SSH 主机密钥所需工具")
    if not inspect_path(image, "/var/lib/wonderduck"):
        modify(image, "mkdir /var/lib/wonderduck")
    unit_path = "/etc/systemd/system/" + SSH_UNIT_NAME
    staged_unit = tempdir / SSH_UNIT_NAME
    staged_unit.write_text(SSH_UNIT, encoding="utf-8")
    staged_script = tempdir / "generate-ssh-host-keys"
    staged_script.write_text(SSH_GENERATOR, encoding="utf-8")
    staged_marker = tempdir / "ssh-keys-pending"
    staged_marker.write_text("", encoding="utf-8")
    for path, content, mode in (("/usr/local/sbin/wonderduck-generate-ssh-host-keys", staged_script, "0100755"),
                                (unit_path, staged_unit, "0100644"),
                                (SSH_MARKER, staged_marker, "0100644")):
        if inspect_path(image, path):
            modify(image, "rm " + path)
        write_file(image, content, path)
        modify(image, "set_inode_field {} mode {}".format(path, mode))
    for service in ("ssh.service", "sshd.service"):
        requires = "/etc/systemd/system/" + service + ".requires"
        if not inspect_path(image, requires):
            modify(image, "mkdir " + requires)
        link = requires + "/" + SSH_UNIT_NAME
        existing = inspect_path(image, link)
        if existing:
            target = re.search(r'Fast link dest: "([^"]+)"', existing)
            if not target or target.group(1) not in ("../" + SSH_UNIT_NAME, unit_path):
                raise RuntimeError("镜像内的 SSH 密钥服务链接指向其他位置")
        else:
            result = run(["debugfs", "-w", "-f", "-", image],
                         input_text="cd {}\nsymlink {} ../{}\n".format(requires, SSH_UNIT_NAME, SSH_UNIT_NAME),
                         capture=True)
            if "\n".join(result.stderr.splitlines()[1:]).strip():
                raise RuntimeError("启用首次 SSH 密钥生成失败")
        if not inspect_path(image, link):
            raise RuntimeError("无法确认镜像中的首次 SSH 密钥生成设置")


def refresh_journal(image):
    details = run(["dumpe2fs", "-h", image], capture=True).stdout
    features = re.search(r"(?m)^Filesystem features:\s*(.*)$", details)
    if not features:
        raise RuntimeError("无法检查文件系统日志")
    if "has_journal" not in features.group(1).split():
        return
    journal, error = debugfs(image, "stat <8>")
    size = re.search(r"\bSize:\s*(\d+)", journal)
    if error or not size or int(size.group(1)) % MIB:
        raise RuntimeError("无法确认文件系统日志大小")
    journal_mib = int(size.group(1)) // MIB
    run(["tune2fs", "-O", "^has_journal", image], capture=True)
    run(["tune2fs", "-j", "-J", "size={}".format(journal_mib), image], capture=True)


def filesystem_geometry(image):
    info = run(["dumpe2fs", "-h", image], capture=True).stdout
    blocks = re.search(r"(?m)^Block count:\s*(\d+)\s*$", info)
    block_size = re.search(r"(?m)^Block size:\s*(\d+)\s*$", info)
    if not blocks or not block_size:
        raise RuntimeError("无法读取 ext4 文件系统大小")
    return int(blocks.group(1)), int(block_size.group(1))


def output_path(directory):
    name = "wonderduck_rk3566_" + datetime.date.today().strftime("%Y%m%d")
    for index in range(1, 10000):
        path = directory / (name + ("" if index == 1 else "_" + str(index)) + ".img")
        outputs = (path, Path(str(path) + ".sha256"), Path(str(path) + ".gz"),
                   Path(str(path) + ".gz.sha256"))
        if not any(item.exists() or item.is_symlink() for item in outputs):
            return path
    raise RuntimeError("当天的备份文件名已用完")


def copy_sparse(source, destination, offset, size):
    with open(source, "rb", buffering=0) as src, open(destination, "r+b", buffering=0) as dst:
        position = 0
        while position < size:
            try:
                start = os.lseek(src.fileno(), position, os.SEEK_DATA)
            except OSError as error:
                if error.errno == errno.ENXIO:
                    break
                raise
            end = min(os.lseek(src.fileno(), start, os.SEEK_HOLE), size)
            src.seek(start)
            dst.seek(offset + start)
            remaining = end - start
            while remaining:
                data = src.read(min(4 * MIB, remaining))
                if not data:
                    raise RuntimeError("读取临时根文件系统时意外结束")
                dst.write(data)
                remaining -= len(data)
            position = end
        dst.flush()
        os.fsync(dst.fileno())


def verify_and_hash(image, root_image, offset, size):
    image_hash, actual_root, expected_root = hashlib.sha256(), hashlib.sha256(), hashlib.sha256()
    with open(image, "rb") as disk, open(root_image, "rb") as root:
        position = 0
        while True:
            data = disk.read(4 * MIB)
            if not data:
                break
            image_hash.update(data)
            start = max(offset - position, 0)
            end = min(offset + size - position, len(data))
            if start < end:
                actual_root.update(data[start:end])
            position += len(data)
        while True:
            data = root.read(4 * MIB)
            if not data:
                break
            expected_root.update(data)
    if actual_root.digest() != expected_root.digest():
        raise RuntimeError("镜像根分区校验失败，备份未完成")
    return image_hash.hexdigest()


def compress_image(image, expected_hash, compressor, owner):
    """Create and verify an additional gzip image without replacing existing files."""
    destination = Path(str(image) + ".gz")
    checksum = Path(str(destination) + ".sha256")
    if any(path.exists() or path.is_symlink() for path in (destination, checksum)):
        raise RuntimeError("压缩镜像或校验文件已存在：" + str(destination))
    size = image.stat().st_size
    if size <= 0:
        raise RuntimeError("原始镜像为空，无法压缩")
    command = [compressor, "-6", "-n", "-c"]
    if Path(compressor).name == "pigz":
        command += ["-p", str(min(4, os.cpu_count() or 1))]
    fd, name = tempfile.mkstemp(prefix="." + destination.name + ".", suffix=".partial", dir=image.parent)
    partial = Path(name)
    checksum_created = False
    published = False
    try:
        compressed_hash = hashlib.sha256()
        print("正在压缩镜像……", flush=True)
        with os.fdopen(fd, "wb") as output, image.open("rb", buffering=0) as source:
            process = subprocess.Popen(command, stdin=source, stdout=subprocess.PIPE)
            last_progress = time.monotonic()
            try:
                while True:
                    data = process.stdout.read1(MIB)
                    if not data:
                        break
                    output.write(data)
                    compressed_hash.update(data)
                    now = time.monotonic()
                    if now - last_progress >= 5:
                        print("压缩进度：{}%".format(min(99, source.tell() * 100 // size)), flush=True)
                        last_progress = now
                process.stdout.close()
                if process.wait() != 0:
                    raise RuntimeError("镜像压缩程序执行失败")
            except BaseException:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                raise
            finally:
                process.stdout.close()
            output.flush()
            os.fsync(output.fileno())
        print("压缩进度：100%；正在校验解压后的镜像……", flush=True)
        restored_hash = hashlib.sha256()
        restored_size = 0
        last_progress = time.monotonic()
        with partial.open("rb") as compressed:
            process = subprocess.Popen([compressor, "-d", "-c"], stdin=compressed, stdout=subprocess.PIPE)
            try:
                while True:
                    data = process.stdout.read(4 * MIB)
                    if not data:
                        break
                    restored_size += len(data)
                    if restored_size > size:
                        raise RuntimeError("压缩镜像解压后超过原始大小")
                    restored_hash.update(data)
                    now = time.monotonic()
                    if now - last_progress >= 5:
                        print("校验进度：{}%".format(min(99, restored_size * 100 // size)), flush=True)
                        last_progress = now
                process.stdout.close()
                if process.wait() != 0:
                    raise RuntimeError("压缩镜像完整性校验失败")
            except BaseException:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                raise
            finally:
                process.stdout.close()
        if restored_size != size or restored_hash.hexdigest() != expected_hash:
            raise RuntimeError("压缩镜像解压后与原始镜像不一致")
        print("校验进度：100%", flush=True)
        if all(value and value.isdigit() for value in owner):
            os.chown(partial, int(owner[0]), int(owner[1]))
        with checksum.open("x", encoding="ascii") as stream:
            checksum_created = True
            stream.write("{}  {}\n".format(compressed_hash.hexdigest(), destination.name))
            stream.flush()
            os.fsync(stream.fileno())
        if all(value and value.isdigit() for value in owner):
            os.chown(checksum, int(owner[0]), int(owner[1]))
        os.link(partial, destination)
        published = True
        partial.unlink()
        print("压缩镜像：{}（{:.2f} GiB）".format(destination, destination.stat().st_size / 2**30), flush=True)
        print("压缩镜像校验文件：{}".format(checksum), flush=True)
    finally:
        if partial.exists():
            partial.unlink()
        if checksum_created and not published:
            checksum.unlink()


def save_table(image, table, root, root_sectors):
    lines = ["label: gpt", "label-id: " + table["id"], "unit: sectors", ""]
    for part in table["partitions"]:
        fields = ["start={}".format(part["start"]),
                  "size={}".format(root_sectors if part is root else part["size"]),
                  "type=" + part["type"], "uuid=" + part["uuid"]]
        for key in ("name", "attrs"):
            if part.get(key):
                value = part[key].replace("\\", "\\\\").replace('"', '\\"')
                fields.append('{}="{}"'.format(key, value))
        lines.append(", ".join(fields))
    with open(image, "rb") as stream:
        boot_code = stream.read(440)
    run(["sfdisk", "--force", "--no-reread", "--wipe", "never", image], input_text="\n".join(lines) + "\n")
    with open(image, "r+b") as stream:
        stream.write(boot_code)
    run(["sfdisk", "--verify", image])


def main():
    parser = argparse.ArgumentParser(description="备份离线 GPT/ext4 TF 卡，输出可直接烧录的小镜像")
    parser.add_argument("source", nargs="?", help="TF 整盘设备或现有磁盘镜像；省略时自动检测可移动磁盘")
    parser.add_argument("--output-dir", default=".", help="输出目录，默认为当前目录")
    parser.add_argument("--reserve-mib", type=int, default=2048, help="最小根文件系统之外预留的 MiB，默认 2048")
    args = parser.parse_args()
    if args.reserve_mib < 0:
        raise ValueError("预留空间不能为负数")
    for command in ("lsblk", "findmnt", "sfdisk", "blkid", "e2image", "e2fsck", "resize2fs", "dumpe2fs", "debugfs", "tune2fs", "umount"):
        if not shutil.which(command):
            raise RuntimeError("缺少必需工具：" + command)
    compressor = shutil.which("pigz") or shutil.which("gzip")
    if not compressor:
        raise RuntimeError("缺少压缩工具：请安装 pigz 或 gzip")
    output_dir = Path(args.output_dir).resolve(strict=True)
    if not output_dir.is_dir():
        raise RuntimeError("输出路径不是目录")
    source, block_device = prepare_source(Path(args.source) if args.source else find_source(), output_dir)
    table, root = partition_table(source, block_device)
    destination = output_path(output_dir)
    print("备份源：{}；目标：{}".format(source, destination), flush=True)
    partial = None
    try:
        with tempfile.TemporaryDirectory(prefix="tf-image-backup-") as temp:
            tempdir = Path(temp)
            filesystem = tempdir / "rootfs.img"
            print("复制已使用的 ext4 数据块，不修改源卡……", flush=True)
            clone = ["e2image", "-r", "-a", "-p"]
            clone += [root["node"], filesystem] if block_device else ["-o", root["start"] * SECTOR, source, filesystem]
            run(clone)
            blocks, block_size = filesystem_geometry(filesystem)
            os.truncate(filesystem, blocks * block_size)
            run(["e2fsck", "-f", "-p", filesystem], allowed=(0, 1))
            print("清理镜像副本中的私人网络、蓝牙配对和 SSH 身份……", flush=True)
            sanitize(filesystem, tempdir, debugfs, write_file)
            install_expand_once(filesystem, tempdir)
            install_bluetooth_reset_once(filesystem, tempdir)
            install_ssh_key_generation(filesystem, tempdir)
            run(["e2fsck", "-f", "-p", filesystem], allowed=(0, 1))
            refresh_journal(filesystem)
            run(["e2fsck", "-f", "-p", "-D", filesystem], allowed=(0, 1))
            cleaned = tempdir / "rootfs-sanitized.img"
            run(["e2image", "-r", "-a", "-p", filesystem, cleaned])
            os.truncate(cleaned, blocks * block_size)
            filesystem.unlink()
            filesystem = cleaned
            run(["e2fsck", "-f", "-n", filesystem])
            previous_target = None
            for _ in range(8):
                run(["resize2fs", "-M", filesystem])
                minimum_blocks, block_size = filesystem_geometry(filesystem)
                reserve_blocks = (args.reserve_mib * MIB + block_size - 1) // block_size
                target_blocks = minimum_blocks + reserve_blocks
                if reserve_blocks:
                    run(["resize2fs", filesystem, str(target_blocks)])
                if target_blocks == previous_target or not reserve_blocks:
                    break
                previous_target = target_blocks
            else:
                raise RuntimeError("文件系统缩放后的最小大小未稳定，备份已停止")
            size = target_blocks * block_size
            os.truncate(filesystem, size)
            run(["e2fsck", "-f", "-n", filesystem])
            print("最小根文件系统 {:.2f} GiB，预留 {:.2f} GiB。".format(
                minimum_blocks * block_size / 2**30, args.reserve_mib / 1024), flush=True)
            offset = root["start"] * SECTOR
            if size % SECTOR:
                raise RuntimeError("根文件系统大小与扇区不对齐")
            total_sectors = ((root["start"] + size // SECTOR + 33 + 2047) // 2048) * 2048
            needed = total_sectors * SECTOR
            if shutil.disk_usage(output_dir).free < needed:
                raise RuntimeError("输出目录可用空间不足，至少需要 {:.2f} GiB".format(needed / 2**30))
            fd, path = tempfile.mkstemp(prefix="." + destination.name + ".", suffix=".partial", dir=output_dir)
            partial = Path(path)
            with os.fdopen(fd, "wb") as out, open(source, "rb") as original:
                remaining = offset
                while remaining:
                    data = original.read(min(4 * MIB, remaining))
                    if not data:
                        raise RuntimeError("读取源磁盘启动区域时意外结束")
                    out.write(data)
                    remaining -= len(data)
                out.truncate(needed)
            save_table(partial, table, root, size // SECTOR)
            print("写入缩小后的根分区并校验……", flush=True)
            copy_sparse(filesystem, partial, offset, size)
            digest = verify_and_hash(partial, filesystem, offset, size)
            owner = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
            if all(value and value.isdigit() for value in owner):
                os.chown(partial, int(owner[0]), int(owner[1]))
            os.link(partial, destination)
            partial.unlink()
            partial = None
            checksum = Path(str(destination) + ".sha256")
            with checksum.open("x", encoding="ascii") as stream:
                stream.write("{}  {}\n".format(digest, destination.name))
            if all(value and value.isdigit() for value in owner):
                os.chown(checksum, int(owner[0]), int(owner[1]))
            print("原始镜像：{}（{:.2f} GiB）".format(destination, needed / 2**30), flush=True)
            print("校验文件：{}".format(checksum), flush=True)
            try:
                compress_image(destination, digest, compressor, owner)
            except BaseException:
                print("压缩未完成，原始镜像仍已保留：{}".format(destination), file=sys.stderr, flush=True)
                raise
            print("备份完成。", flush=True)
    finally:
        if partial and partial.exists():
            partial.unlink()


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print("备份失败：{}".format(error), file=sys.stderr)
        sys.exit(1)
