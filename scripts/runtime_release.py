#!/usr/bin/env python3
"""Build and install signed ARM64 runtime bundles for the existing device image."""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import posixpath
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
from urllib.parse import quote
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BASE = 'https://github.com/Hiwonder/wonderduck/releases/download'
MAIN_URL = 'https://github.com/Hiwonder/wonderduck.git'
GITEE_BASE = 'https://gitee.com/hiwonder_official/wonderduck/releases/download'
GITEE_MAIN_URL = 'https://gitee.com/hiwonder_official/wonderduck.git'
PUBLIC_KEY = Path('deploy/system/runtime-release.pub')
DEVICE_VERSION = Path('/etc/robot/device-version.json')
ROBOT_LOCK = Path('/run/robotd.sock.lock')
SYSTEMD_ROOT = Path('/home/wonderduck/wonderduck')
MOUNT_INFO = Path('/proc/self/mountinfo')
SYSTEMD_SOURCE = Path('deploy/systemd/robotd.service.d/source.conf')
SYSTEMD_DROPIN = Path('/etc/systemd/system/robotd.service.d/source.conf')
PADD_SOURCE = Path('deploy/systemd/padd.service.d/source.conf')
PADD_DROPIN = Path('/etc/systemd/system/padd.service.d/source.conf')
WEBPADD_SOURCE = Path('deploy/systemd/webpadd.service.d/recording.conf')
WEBPADD_DROPIN = Path('/etc/systemd/system/webpadd.service.d/recording.conf')
PROMPT_SERVICE_SOURCE = Path('deploy/systemd/wonderduck-prompt-sounds.service')
PROMPT_SERVICE_TARGET = Path('/etc/systemd/system/wonderduck-prompt-sounds.service')
LOGIN_HELP_SOURCE = Path('deploy/system/10-help-text')
LOGIN_HELP_TARGET = Path('/etc/update-motd.d/10-help-text')
LOGIN_HEADER_SOURCE = Path('deploy/system/00-header')
LOGIN_HEADER_TARGET = Path('/etc/update-motd.d/00-header')
BLUETOOTH_TRUST_SOURCE = Path('scripts/bluetooth_auto_trust.py')
BLUETOOTH_TRUST_TARGET = Path('/usr/local/libexec/robot-bluetooth-auto-trust.py')
BLUETOOTH_SIXAXIS_SOURCE = Path('scripts/bluetooth_sixaxis.py')
BLUETOOTH_SIXAXIS_TARGET = Path('/usr/local/libexec/bluetooth_sixaxis.py')
USBMOUNT_GUARD_SOURCE = Path('deploy/system/usbmount-if-needed')
USBMOUNT_GUARD_TARGET = Path('/usr/local/libexec/usbmount-if-needed')
USBMOUNT_DROPIN_SOURCE = Path('deploy/systemd/usbmount@.service.d/skip-mounted.conf')
USBMOUNT_DROPIN_TARGET = Path('/etc/systemd/system/usbmount@.service.d/skip-mounted.conf')
CAMERA_SCRIPT = Path('scripts/run-web-camera.sh')
CAMERA_LAUNCHER = Path('target/release/camera-launch')
CAMERA_LINK_TARGET = '../../scripts/run-web-camera.sh'
MEDIA_RUN_SOURCE = Path('deploy/system/media-runtime-run')
ARCHIVE_NAME = 'aarch64.tar.gz'
MANIFEST_NAME = 'manifest.json'
SIGNATURE_NAME = 'manifest.json.sig'
MAX_ARCHIVE = 1024 * 1024 * 1024
ROBOTD = 'target/release/robotd'
PADD = 'target/release/padd'
WEBPADD = 'target/release/webpadd'
STUDIO_WEB = 'extensions/wonderduck_studio/console/web/build'
MEDIA_ROOT = 'media-runtime-candidate'
ONNX_RUNTIME = 'onnxruntime-linux-aarch64-1.28.0'
OBJECT_MODEL = 'models/object-detection.rknn'
RKNN_RUNTIME = 'models/librknnrt.so'
STARTUP_SOUND = 'target/prompt-sounds/startup.wav'
LOW_BATTERY_SOUND = 'target/prompt-sounds/low_battery.wav'
SOUND_COMPONENTS = (STARTUP_SOUND, LOW_BATTERY_SOUND)
COMPONENTS = (ROBOTD, PADD, WEBPADD, STUDIO_WEB, MEDIA_ROOT, ONNX_RUNTIME, OBJECT_MODEL, RKNN_RUNTIME) + SOUND_COMPONENTS
DIRECTORIES = frozenset((STUDIO_WEB, MEDIA_ROOT, ONNX_RUNTIME))
MEDIA_COMPONENTS = tuple('media-runtime-candidate/' + name for name in
                         ('bin', 'lib', 'libexec', 'run'))
REPLACEABLE_DIRECTORIES = frozenset((STUDIO_WEB, ONNX_RUNTIME) + MEDIA_COMPONENTS[:3])
OBSOLETE_MEDIA_METADATA = ('BUILD.json', 'SHA256SUMS')
# Camera caches and lock files are device state, not package contents.
# Install the web entry last, after its backend and other resources.
INSTALL_COMPONENTS = (ROBOTD, PADD, WEBPADD) + MEDIA_COMPONENTS + (ONNX_RUNTIME, OBJECT_MODEL, RKNN_RUNTIME) + SOUND_COMPONENTS + (STUDIO_WEB,)
# Only the managed RKNN runtime library may be replaced inside models/.
PRIVILEGED_REPLACE_COMPONENTS = frozenset(
    (ROBOTD, PADD, WEBPADD) + MEDIA_COMPONENTS + (ONNX_RUNTIME, RKNN_RUNTIME) + SOUND_COMPONENTS
    + (STUDIO_WEB, CAMERA_LAUNCHER.as_posix()))
PRIVILEGED_NEW_PARENTS = frozenset(('target/release', 'target/prompt-sounds', MEDIA_ROOT,
                                    'extensions/wonderduck_studio/console/web'))
OBSOLETE_DEBUG_FILES = ('target/debug/webpadd', 'target/debug/padd')
PRIVILEGED_OBSOLETE_FILES = frozenset(
    tuple(MEDIA_ROOT + '/' + name for name in OBSOLETE_MEDIA_METADATA) + OBSOLETE_DEBUG_FILES)
MARKER = '.runtime-release.json'
TRANSACTION_PREFIX = '.runtime-transaction-'
VERSION_PATTERN = re.compile(r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\Z')
RELEASE_VERSION_PATTERN = re.compile(
    r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(?:-dev\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*))?\Z')
REVISION_PATTERN = re.compile(r'[0-9a-f]{40}\Z')
SHA_PATTERN = re.compile(r'[0-9a-f]{64}\Z')


def version(value):
    if not isinstance(value, str) or not VERSION_PATTERN.fullmatch(value):
        raise ValueError('版本必须为三段数字，例如 0.1.0')
    return tuple(int(part) for part in value.split('.'))


def development_release(value):
    match = RELEASE_VERSION_PATTERN.fullmatch(value) if isinstance(value, str) else None
    if match is None:
        raise ValueError('发布版本格式应为 0.1.0 或 0.1.0-dev.1.1')
    return match.group(4) is not None


def digest_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout.strip()


def official_source(remote):
    for name, repository, base in (('github', MAIN_URL, DEFAULT_BASE),
                                   ('gitee', GITEE_MAIN_URL, GITEE_BASE)):
        addresses = {repository, repository[:-4]} if repository.endswith('.git') else {repository}
        if remote in addresses:
            return name, base
    raise ValueError('WonderDuck 仓库地址无效')


def release_download_base(repo, override=None):
    if override:
        return override
    return official_source(git(repo, 'remote', 'get-url', 'origin'))[1]


def require_release_branch(repo, revision, branch):
    if branch not in ('main', 'develop'):
        raise ValueError('发布分支无效')
    official_source(git(repo, 'remote', 'get-url', 'origin'))
    try:
        git(repo, 'fetch', '--no-tags', '--no-recurse-submodules', '--append', 'origin',
            '+refs/heads/{0}:refs/remotes/origin/{0}'.format(branch))
    except subprocess.CalledProcessError as error:
        raise ValueError('无法获取官方 {} 分支'.format(branch)) from error
    result = subprocess.run(['git', '-C', str(repo), 'merge-base', '--is-ancestor', revision,
                             'refs/remotes/origin/' + branch], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        raise ValueError('该发布版本不属于官方 {} 分支'.format(branch))


def require_main_release(repo, revision):
    require_release_branch(repo, revision, 'main')


def revisions(repo):
    studio = repo / 'extensions/wonderduck_studio'
    for directory in (repo, studio):
        if git(directory, 'status', '--porcelain', '--untracked-files=no'):
            raise ValueError('源码仓库存在未提交的修改：' + str(directory))
    main_revision = git(repo, 'rev-parse', 'HEAD')
    studio_revision = git(studio, 'rev-parse', 'HEAD')
    if studio_revision != git(repo, 'rev-parse', 'HEAD:extensions/wonderduck_studio'):
        raise ValueError('Studio 子模块版本与主仓库记录不一致')
    return main_revision, studio_revision


def require_arm64(path):
    with path.open('rb') as stream:
        header = stream.read(20)
    if (len(header) < 20 or header[:4] != b'\x7fELF' or header[4] != 2
            or header[5] != 1 or int.from_bytes(header[18:20], 'little') != 183):
        raise ValueError('不是 ARM64 Linux 可执行文件或动态库：' + str(path))


def package(args):
    development = development_release(args.version)
    version(args.min_hardware)
    version(args.min_image)
    root = args.repo.resolve()
    main_revision, studio_revision = revisions(root)
    require_release_branch(root, main_revision, 'develop' if development else 'main')
    binary_dir = args.binaries.resolve()
    web = args.web_build.resolve()
    assets = args.runtime_assets.resolve()
    key = args.signing_key.resolve()
    if not key.is_file():
        raise ValueError('找不到发布签名私钥')
    for binary in ('robotd', 'padd', 'webpadd'):
        path = binary_dir / binary
        require_arm64(path)
        if main_revision.encode('ascii') not in path.read_bytes():
            raise ValueError('程序未嵌入当前源码版本，请重新构建：' + str(path))
    for file in (web / 'index.html', assets / 'models/object-detection.rknn',
                 assets / 'models/librknnrt.so',
                 assets / 'onnxruntime-linux-aarch64-1.28.0/lib/libonnxruntime.so.1.28.0',
                 root / MEDIA_RUN_SOURCE,
                 root / 'assets/prompt-sounds/startup.wav',
                 root / 'assets/prompt-sounds/low_battery.wav'):
        if not file.is_file():
            raise ValueError('缺少运行资源：' + str(file))
    require_arm64(assets / 'models/librknnrt.so')
    require_arm64(assets / 'onnxruntime-linux-aarch64-1.28.0/lib/libonnxruntime.so.1.28.0')
    require_arm64(assets / 'media-runtime-candidate/bin/mediad')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='runtime-release-') as folder:
        payload = Path(folder) / 'payload'
        for binary in ('robotd', 'padd', 'webpadd'):
            destination = payload / 'target/release' / binary
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(binary_dir / binary, destination)
        destination = payload / STUDIO_WEB
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(web, destination, symlinks=True)
        media_source = assets / 'media-runtime-candidate'
        media_dest = payload / 'media-runtime-candidate'
        media_dest.mkdir()
        (media_dest / 'bin').mkdir()
        shutil.copy2(media_source / 'bin/mediad', media_dest / 'bin/mediad')
        for name in ('lib', 'libexec'):
            shutil.copytree(media_source / name, media_dest / name, symlinks=True)
        shutil.copy2(root / MEDIA_RUN_SOURCE, media_dest / 'run')
        shutil.copytree(assets / 'onnxruntime-linux-aarch64-1.28.0',
                        payload / 'onnxruntime-linux-aarch64-1.28.0', symlinks=True)
        (payload / 'models').mkdir()
        for name in ('object-detection.rknn', 'librknnrt.so'):
            shutil.copy2(assets / 'models' / name, payload / 'models' / name)
        sounds = payload / 'target/prompt-sounds'
        sounds.mkdir(parents=True, exist_ok=True)
        for name in ('startup.wav', 'low_battery.wav'):
            shutil.copy2(root / 'assets/prompt-sounds' / name, sounds / name)
        archive = output / ARCHIVE_NAME
        with tarfile.open(str(archive), 'w:gz', compresslevel=4) as tar:
            tar.add(str(payload), arcname='payload', recursive=True)
    manifest = {
        'schema': 1,
        'tag': 'v' + args.version,
        'version': args.version,
        'architecture': 'aarch64',
        'source_revision': main_revision,
        'studio_revision': studio_revision,
        'min_hardware_version': args.min_hardware,
        'min_image_version': args.min_image,
        'archive': ARCHIVE_NAME,
        'size': archive.stat().st_size,
        'sha256': digest_file(archive),
    }
    validate_manifest(manifest, manifest['tag'])
    path = output / MANIFEST_NAME
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    subprocess.run(['openssl', 'dgst', '-sha256', '-sign', str(key), '-out',
                    str(output / SIGNATURE_NAME), str(path)], check=True)
    verify_signature(root / PUBLIC_KEY, path, output / SIGNATURE_NAME)
    print('发布包：{}（{} 字节）'.format(archive, manifest['size']))
    print('源码：{}，Studio：{}'.format(main_revision, studio_revision))
    print('SHA-256：' + manifest['sha256'])
    print('GitHub Release title: v' + args.version)
    print('GitHub Release type: ' + ('pre-release' if development else 'release'))


def validate_manifest(data, tag):
    expected = {'schema', 'tag', 'version', 'architecture', 'source_revision', 'studio_revision',
                'min_hardware_version', 'min_image_version', 'archive', 'size', 'sha256'}
    if not isinstance(data, dict) or set(data) != expected or data['schema'] != 1:
        raise ValueError('发布清单格式不正确')
    development_release(data['version'])
    version(data['min_hardware_version'])
    version(data['min_image_version'])
    if data['tag'] != tag or tag != 'v' + data['version']:
        raise ValueError('发布标签与版本不一致')
    if data['architecture'] != 'aarch64' or data['archive'] != ARCHIVE_NAME:
        raise ValueError('不支持的运行包架构或文件名')
    if not all(isinstance(data[name], str) and REVISION_PATTERN.fullmatch(data[name])
               for name in ('source_revision', 'studio_revision')):
        raise ValueError('发布清单中的源码版本无效')
    if not isinstance(data['sha256'], str) or not SHA_PATTERN.fullmatch(data['sha256']):
        raise ValueError('发布清单中的摘要无效')
    if type(data['size']) is not int or not 0 < data['size'] <= MAX_ARCHIVE:
        raise ValueError('发布清单中的包大小无效')


def verify_signature(public_key, manifest, signature):
    result = subprocess.run(['openssl', 'dgst', '-sha256', '-verify', str(public_key),
                             '-signature', str(signature), str(manifest)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        raise ValueError('发布清单签名校验失败')


def download(url, destination, limit, expected_size=None):
    total = 0
    with urlopen(url, timeout=60) as response, destination.open('xb') as stream:
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if total > limit:
                raise ValueError('下载文件超过允许大小')
            stream.write(chunk)
    if expected_size is not None and total != expected_size:
        raise ValueError('运行包大小与发布清单不一致')


def allowed_path(name):
    if name == 'payload':
        return True
    if not name.startswith('payload/'):
        return False
    relative = name[len('payload/'):]
    return any(relative == component or relative.startswith(component + '/')
               or component.startswith(relative + '/') for component in COMPONENTS)


def unpack(archive, stage):
    """Extract regular files and internal symlinks without following archive links."""
    with tarfile.open(str(archive), 'r:gz') as tar:
        members = tar.getmembers()
        seen = set()
        symlinks = set()
        total = 0
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or str(path) != member.name
                    or not allowed_path(member.name) or member.name in seen):
                raise ValueError('运行包包含无效路径：' + member.name)
            seen.add(member.name)
            if member.issym():
                target = posixpath.normpath(posixpath.join(posixpath.dirname(member.name), member.linkname))
                if member.linkname.startswith('/') or not target.startswith('payload/'):
                    raise ValueError('运行包包含越界链接：' + member.name)
                owner = next((part for part in DIRECTORIES
                              if member.name.startswith('payload/' + part + '/')), None)
                if owner is None or not target.startswith('payload/' + owner + '/'):
                    raise ValueError('运行包链接超出资源目录：' + member.name)
                symlinks.add(member.name)
            elif member.isfile():
                total += member.size
                if member.size < 0 or total > MAX_ARCHIVE * 2:
                    raise ValueError('运行包解压后过大')
            elif not member.isdir():
                raise ValueError('运行包包含不支持的文件类型：' + member.name)
        for name in seen:
            path = PurePosixPath(name)
            if any(str(parent) in symlinks for parent in path.parents):
                raise ValueError('运行包文件位于链接目录下：' + name)
        for member in members:
            target = stage.joinpath(*PurePosixPath(member.name).parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                with tar.extractfile(member) as source, target.open('xb') as destination:
                    shutil.copyfileobj(source, destination, 1024 * 1024)
                    destination.flush()
                    os.fchmod(destination.fileno(), 0o755 if member.mode & 0o111 else 0o644)
                    os.fsync(destination.fileno())
        for member in members:
            if member.issym():
                target = stage.joinpath(*PurePosixPath(member.name).parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.symlink_to(member.linkname)
        payload = stage / 'payload'
        for component in COMPONENTS:
            target = payload / component
            if target.is_symlink() or (not target.is_dir() if component in DIRECTORIES else not target.is_file()):
                raise ValueError('运行包缺少必要文件：' + component)
        for component in (ROBOTD, PADD, WEBPADD):
            require_arm64(payload / component)
        for component in MEDIA_COMPONENTS:
            target = payload / component
            expected_dir = component.rsplit('/', 1)[-1] in ('bin', 'lib', 'libexec')
            if target.is_symlink() or (not target.is_dir() if expected_dir else not target.is_file()):
                raise ValueError('运行包缺少摄像头运行文件：' + component)
        if not (payload / STUDIO_WEB / 'index.html').is_file():
            raise ValueError('运行包缺少网页入口')
        for directory, _, _ in os.walk(str(payload), topdown=False, followlinks=False):
            sync_directory(Path(directory))
        sync_directory(stage)


def fingerprint(path):
    digest = hashlib.sha256()

    def visit(item, name):
        info = item.lstat()
        digest.update(name.encode('utf-8') + b'\0')
        if stat.S_ISLNK(info.st_mode):
            digest.update(b'L' + os.readlink(str(item)).encode('utf-8') + b'\0')
        elif stat.S_ISDIR(info.st_mode):
            digest.update(b'D')
            for child in sorted(item.iterdir(), key=lambda value: value.name):
                visit(child, name + '/' + child.name)
        elif stat.S_ISREG(info.st_mode):
            digest.update(b'F' + str(info.st_mode & 0o777).encode('ascii') + b'\0')
            with item.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
        elif stat.S_ISSOCK(info.st_mode):
            digest.update(b'S')
        else:
            raise ValueError('运行文件包含不支持的文件类型：' + str(item))

    visit(path, '.')
    return digest.hexdigest()


def exists(path):
    return path.exists() or path.is_symlink()


def sync_directory(path):
    descriptor = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_journal(stage, state):
    temporary = stage / 'journal.tmp'
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(state, stream, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(stage / 'journal.json'))
    sync_directory(stage)


def write_marker(root, stage, manifest, status):
    temporary = stage / 'marker.tmp'
    marker = dict(manifest, install_status=status)
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(marker, stream, indent=2, sort_keys=True)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(root / MARKER))
    sync_directory(root)


def check_incomplete_version(root, requested):
    marker = root / MARKER
    if not exists(marker):
        return
    if marker.is_symlink() or not marker.is_file():
        raise ValueError('安装状态文件无效：' + str(marker))
    data = json.loads(marker.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('安装状态文件格式无效')
    if data.get('install_status') == 'installing' and data.get('version') != requested:
        raise ValueError('上次安装未完成，请先重新安装版本 {}'.format(data.get('version', '未知')))


def safe_parent(root, destination):
    parent = root
    for part in destination.parent.relative_to(root).parts:
        parent = parent / part
        if parent.is_symlink():
            raise ValueError('安装路径不能包含符号链接：' + str(parent))
        try:
            parent.mkdir(exist_ok=True)
        except PermissionError:
            create_privileged_parent(root, parent)


def create_privileged_parent(root, parent):
    """Create only a missing managed parent, leaving existing ownership alone."""
    try:
        name = parent.relative_to(root).as_posix()
    except ValueError as error:
        raise ValueError('安装目录超出运行目录：' + str(parent)) from error
    if root != SYSTEMD_ROOT or name not in PRIVILEGED_NEW_PARENTS:
        raise ValueError('不允许提权创建非运行包目录：' + str(parent))
    code = r'''
import os, re, sys
from pathlib import Path
root, name = sys.argv[1:]
if root != {root!r} or name not in {names!r}:
    raise ValueError('invalid runtime parent')
target = Path(root) / name
encoded_root, encoded_target = os.fsencode(root), os.fsencode(str(target))
with open('/proc/self/mountinfo', 'rb') as stream:
    for line in stream:
        fields = line.split()
        if len(fields) < 5:
            raise ValueError('invalid mount information')
        mount = re.sub(rb'\\([0-7]{{3}})', lambda m: bytes([int(m[1], 8)]), fields[4])
        if (mount == encoded_target or (mount.startswith(encoded_root + b'/')
                and encoded_target.startswith(mount + b'/'))):
            raise ValueError('runtime parent is inside a mount point')
flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
fd = os.open('/', flags)
try:
    for part in target.parent.parts[1:]:
        next_fd = os.open(part, flags, dir_fd=fd)
        os.close(fd)
        fd = next_fd
    os.mkdir(target.name, 0o755, dir_fd=fd)
    child = os.open(target.name, flags, dir_fd=fd)
    try:
        os.fchown(child, {uid!r}, {gid!r})
        os.fchmod(child, 0o755)
    finally:
        os.close(child)
finally:
    os.close(fd)
'''.format(root=str(SYSTEMD_ROOT), names=tuple(sorted(PRIVILEGED_NEW_PARENTS)),
           uid=os.getuid(), gid=os.getgid())
    try:
        result = subprocess.run(privileged_prefix() + [sys.executable, '-c', code, str(root), name],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as error:
        raise ValueError('无法创建运行目录 {}：{}'.format(parent, error)) from error
    if result.returncode:
        detail = result.stderr.strip()
        raise ValueError('无法创建运行目录 {}{}'.format(parent, '：' + detail if detail else ''))


def cleanup_stage(stage, root):
    if (stage.parent != root or stage.is_symlink() or not stage.is_dir()
            or stage.stat().st_uid != os.getuid()
            or not re.fullmatch(r'\.runtime-transaction-[A-Za-z0-9_]+', stage.name)):
        raise ValueError('安装事务目录异常：' + str(stage))
    if exists(stage / 'old'):
        raise ValueError('发现旧版安装事务，请先检查：' + str(stage))
    shutil.rmtree(stage)


def pending(root):
    result = []
    for stage in sorted(root.glob(TRANSACTION_PREFIX + '*')):
        if stage.is_symlink() or not stage.is_dir() or stage.stat().st_uid != os.getuid():
            raise ValueError('安装事务目录异常：' + str(stage))
        journal = stage / 'journal.json'
        if journal.is_symlink():
            raise ValueError('安装事务记录无效：' + str(stage))
        if not journal.exists():
            # A power loss before the first atomic journal write leaves only its
            # temporary file; no installed path has been touched yet.
            entries = list(stage.iterdir())
            if any(item.name != 'journal.tmp' or item.is_symlink() or not item.is_file()
                   for item in entries):
                raise ValueError('无法读取安装事务：' + str(stage))
            state = {'schema': 2, 'status': 'preparing', 'version': None}
        else:
            try:
                state = json.loads(journal.read_text(encoding='utf-8'))
            except (OSError, ValueError) as error:
                raise ValueError('无法读取安装事务：' + str(stage)) from error
        result.append((stage, state))
    return result


def discard_pending(stage, root, state, requested):
    if not isinstance(state, dict) or state.get('schema') != 2:
        raise ValueError('安装事务记录无效：' + str(stage))
    status = state.get('status')
    if status not in ('preparing', 'applying', 'committed'):
        raise ValueError('安装事务状态无效：' + str(stage))
    if status == 'applying' and state.get('version') != requested:
        raise ValueError('上次安装未完成，请先重新安装版本 {}'.format(state.get('version', '未知')))
    cleanup_stage(stage, root)


def check_mounted_directory(root, destination):
    """Reject mounts inside or below the root on the path to a managed tree."""
    encoded_root, encoded = os.fsencode(str(root)), os.fsencode(str(destination))
    try:
        with MOUNT_INFO.open('rb') as stream:
            for line in stream:
                fields = line.split()
                if len(fields) < 5:
                    raise ValueError('挂载信息格式无效')
                mount = re.sub(rb'\\([0-7]{3})', lambda match: bytes([int(match[1], 8)]), fields[4])
                if (mount == encoded or mount.startswith(encoded + b'/')
                        or (mount.startswith(encoded_root + b'/') and encoded.startswith(mount + b'/'))):
                    raise ValueError('运行目录包含挂载点：' + str(destination))
    except OSError as error:
        raise ValueError('无法检查运行目录挂载状态：' + str(destination)) from error


def remove_privileged_directory(root, destination):
    """Remove only a managed runtime directory left root-owned by old installs."""
    try:
        name = destination.relative_to(root).as_posix()
    except ValueError as error:
        raise ValueError('安装目标超出运行目录：' + str(destination)) from error
    if root != SYSTEMD_ROOT or name not in REPLACEABLE_DIRECTORIES:
        raise ValueError('不允许清理非运行包目录：' + str(destination))

    # Resolve each parent using O_NOFOLLOW and keep the directory fd open until
    # rmtree finishes.  The fd-anchored rmtree does not follow links in the tree.
    # Reject nested mount points, including same-device bind mounts, so root
    # cannot remove files mounted from outside this managed directory.
    code = r'''
import os, re, shutil, stat, sys
from pathlib import Path
root, name = sys.argv[1:]
if root != {root!r} or name not in {names!r}:
    raise ValueError('invalid runtime directory')
target = Path(root) / name
encoded_root, encoded = os.fsencode(root), os.fsencode(str(target))
with open('/proc/self/mountinfo', 'rb') as stream:
    for line in stream:
        fields = line.split()
        if len(fields) < 5:
            raise ValueError('invalid mount information')
        mount = re.sub(rb'\\([0-7]{{3}})', lambda m: bytes([int(m[1], 8)]), fields[4])
        if (mount == encoded or mount.startswith(encoded + b'/')
                or (mount.startswith(encoded_root + b'/') and encoded.startswith(mount + b'/'))):
            raise ValueError('runtime directory contains a mount point')
flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
fd = os.open('/', flags)
try:
    for part in target.parent.parts[1:]:
        next_fd = os.open(part, flags, dir_fd=fd)
        os.close(fd)
        fd = next_fd
    entry = os.stat(target.name, dir_fd=fd, follow_symlinks=False)
    if not stat.S_ISDIR(entry.st_mode):
        raise ValueError('runtime directory is not a directory')
    if not shutil.rmtree.avoids_symlink_attacks:
        raise ValueError('safe directory removal is unavailable')
    shutil.rmtree('/proc/self/fd/' + str(fd) + '/' + target.name)
finally:
    os.close(fd)
'''.format(root=str(SYSTEMD_ROOT), names=tuple(sorted(REPLACEABLE_DIRECTORIES)))
    try:
        result = subprocess.run(privileged_prefix() + [sys.executable, '-c', code, str(root), name],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as error:
        raise ValueError('无法清理旧版运行目录 {}：{}'.format(destination, error)) from error
    if result.returncode:
        detail = result.stderr.strip()
        raise ValueError('无法清理旧版运行目录 {}{}'.format(
            destination, '：' + detail if detail else ''))


def replace_privileged_component(root, source, destination):
    """Move one staged runtime component into a root-owned managed parent."""
    try:
        name = destination.relative_to(root).as_posix()
        source_parts = source.relative_to(root).parts
    except ValueError as error:
        raise ValueError('安装源或目标超出运行目录') from error
    expected_parts = PurePosixPath(name).parts
    is_camera = name == CAMERA_LAUNCHER.as_posix()
    valid_source = (len(source_parts) == 2 and source_parts[1] == 'camera-launch.tmp' if is_camera else
                    len(source_parts) == len(expected_parts) + 2 and source_parts[1] == 'payload'
                    and source_parts[2:] == expected_parts)
    if (root != SYSTEMD_ROOT or name not in PRIVILEGED_REPLACE_COMPONENTS
            or not source_parts or not valid_source
            or not re.fullmatch(r'\.runtime-transaction-[A-Za-z0-9_]+', source_parts[0])
            ):
        raise ValueError('不允许提权替换非运行包文件：' + str(destination))

    # Open both paths relative to directory fds, without traversing symlinks.
    # The staged source must belong to the invoking user and match the exact
    # destination. This never grants write access to the parent directory.
    code = r'''
import os, re, stat, sys
from pathlib import Path
root, name, transaction = sys.argv[1:]
if (root != {root!r} or name not in {names!r}
        or not re.fullmatch(r'\.runtime-transaction-[A-Za-z0-9_]+', transaction)):
    raise ValueError('invalid runtime component')
target = Path(root) / name
encoded_root, encoded_target = os.fsencode(root), os.fsencode(str(target))
with open('/proc/self/mountinfo', 'rb') as stream:
    for line in stream:
        fields = line.split()
        if len(fields) < 5:
            raise ValueError('invalid mount information')
        mount = re.sub(rb'\\([0-7]{{3}})', lambda m: bytes([int(m[1], 8)]), fields[4])
        if (mount == encoded_target or (mount.startswith(encoded_root + b'/')
                and encoded_target.startswith(mount + b'/'))):
            raise ValueError('runtime component is inside a mount point')
flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
fds = []
def open_at(parent, name):
    fd = os.open(name, flags, dir_fd=parent)
    fds.append(fd)
    return fd
try:
    fd = os.open('/', flags)
    fds.append(fd)
    for part in Path(root).parts[1:]:
        fd = open_at(fd, part)
    source_fd = open_at(fd, transaction)
    if os.fstat(source_fd).st_uid != {uid!r}:
        raise ValueError('invalid transaction owner')
    is_camera = name == {camera!r}
    if not is_camera:
        source_fd = open_at(source_fd, 'payload')
    destination_fd = fd
    for part in Path(name).parts[:-1]:
        if not is_camera:
            source_fd = open_at(source_fd, part)
        destination_fd = open_at(destination_fd, part)
    leaf = Path(name).name
    source_leaf = 'camera-launch.tmp' if is_camera else leaf
    item = os.stat(source_leaf, dir_fd=source_fd, follow_symlinks=False)
    is_directory = name in {directories!r}
    valid_type = (stat.S_ISLNK(item.st_mode) and os.readlink(source_leaf, dir_fd=source_fd) == {link!r}
                  if is_camera else stat.S_ISDIR(item.st_mode) if is_directory else stat.S_ISREG(item.st_mode))
    if item.st_uid != {uid!r} or not valid_type:
        raise ValueError('invalid staged runtime component')
    try:
        current = os.stat(leaf, dir_fd=destination_fd, follow_symlinks=False)
    except FileNotFoundError:
        current = None
    if current is not None and (is_directory or not (stat.S_ISLNK(current.st_mode)
                                                     if is_camera else stat.S_ISREG(current.st_mode))):
        raise ValueError('invalid runtime destination')
    os.replace(source_leaf, leaf, src_dir_fd=source_fd, dst_dir_fd=destination_fd)
finally:
    for descriptor in reversed(fds):
        os.close(descriptor)
'''.format(root=str(SYSTEMD_ROOT), names=tuple(sorted(PRIVILEGED_REPLACE_COMPONENTS)),
           directories=tuple(sorted(REPLACEABLE_DIRECTORIES)), uid=os.getuid(),
           camera=CAMERA_LAUNCHER.as_posix(), link=CAMERA_LINK_TARGET)
    try:
        result = subprocess.run(privileged_prefix() + [sys.executable, '-c', code,
                                str(root), name, source_parts[0]],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as error:
        raise ValueError('无法安装运行文件 {}：{}'.format(destination, error)) from error
    if result.returncode:
        detail = result.stderr.strip()
        raise ValueError('无法安装运行文件 {}{}'.format(
            destination, '：' + detail if detail else ''))


def replace_component(root, source, destination):
    safe_parent(root, destination)
    if destination.is_symlink():
        raise ValueError('安装目标不能是符号链接：' + str(destination))
    if exists(destination):
        if source.is_dir() and destination.is_dir():
            if destination.relative_to(root).as_posix() not in REPLACEABLE_DIRECTORIES:
                raise ValueError('不允许清理非运行包目录：' + str(destination))
            check_mounted_directory(root, destination)
            try:
                shutil.rmtree(destination)
            except PermissionError:
                remove_privileged_directory(root, destination)
        elif not source.is_file() or not destination.is_file():
            raise ValueError('安装目标类型不匹配：' + str(destination))
    try:
        os.replace(str(source), str(destination))
    except PermissionError:
        replace_privileged_component(root, source, destination)
    sync_directory(destination.parent)


def remove_privileged_file(root, destination):
    """Remove only the known obsolete runtime artifacts, never user files."""
    try:
        name = destination.relative_to(root).as_posix()
    except ValueError as error:
        raise ValueError('旧运行文件超出运行目录：' + str(destination)) from error
    if root != SYSTEMD_ROOT or name not in PRIVILEGED_OBSOLETE_FILES:
        raise ValueError('不允许清理非运行包文件：' + str(destination))
    code = r'''
import os, re, stat, sys
from pathlib import Path
root, name = sys.argv[1:]
if root != {root!r} or name not in {names!r}:
    raise ValueError('invalid obsolete runtime file')
target = Path(root) / name
encoded_root, encoded_target = os.fsencode(root), os.fsencode(str(target))
with open('/proc/self/mountinfo', 'rb') as stream:
    for line in stream:
        fields = line.split()
        if len(fields) < 5:
            raise ValueError('invalid mount information')
        mount = re.sub(rb'\\([0-7]{{3}})', lambda m: bytes([int(m[1], 8)]), fields[4])
        if (mount == encoded_target or (mount.startswith(encoded_root + b'/')
                and encoded_target.startswith(mount + b'/'))):
            raise ValueError('obsolete runtime file is inside a mount point')
flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
fd = os.open('/', flags)
try:
    for part in target.parent.parts[1:]:
        next_fd = os.open(part, flags, dir_fd=fd)
        os.close(fd)
        fd = next_fd
    if not stat.S_ISREG(os.stat(target.name, dir_fd=fd, follow_symlinks=False).st_mode):
        raise ValueError('obsolete runtime file is not regular')
    os.unlink(target.name, dir_fd=fd)
finally:
    os.close(fd)
'''.format(root=str(SYSTEMD_ROOT), names=tuple(sorted(PRIVILEGED_OBSOLETE_FILES)))
    try:
        result = subprocess.run(privileged_prefix() + [sys.executable, '-c', code, str(root), name],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as error:
        raise ValueError('无法清理旧版运行文件 {}：{}'.format(destination, error)) from error
    if result.returncode:
        detail = result.stderr.strip()
        raise ValueError('无法清理旧版运行文件 {}{}'.format(
            destination, '：' + detail if detail else ''))


def install_missing_model(root, source, destination):
    """Supply a default detection model only when the device has no model yet."""
    safe_parent(root, destination)
    if exists(destination):
        if destination.is_symlink() or destination.is_file():
            return False
        raise ValueError('已有检测模型不是文件：' + str(destination))
    try:
        # The staging directory is created on the installation filesystem.
        # link() atomically refuses to replace a file created by another process.
        os.link(str(source), str(destination), follow_symlinks=False)
    except FileExistsError:
        if destination.is_symlink() or destination.is_file():
            return False
        raise ValueError('已有检测模型不是文件：' + str(destination))
    sync_directory(destination.parent)
    return True


def check_camera_launcher(root):
    script = root / CAMERA_SCRIPT
    if (script.is_symlink() or not script.is_file() or script.resolve() != script
            or not os.access(str(script), os.X_OK)):
        raise ValueError('摄像头启动脚本不存在或不可执行：' + str(script))
    launcher = root / CAMERA_LAUNCHER
    for directory in (root / 'target', launcher.parent):
        if directory.is_symlink() or (directory.exists() and not directory.is_dir()):
            raise ValueError('摄像头启动路径无效：' + str(directory))
    if exists(launcher) and not launcher.is_symlink():
        raise ValueError('摄像头启动入口不是符号链接，请检查：' + str(launcher))


def sync_camera_launcher(root, stage):
    check_camera_launcher(root)
    launcher = root / CAMERA_LAUNCHER
    safe_parent(root, launcher)
    if launcher.is_symlink() and os.readlink(str(launcher)) == CAMERA_LINK_TARGET:
        return
    temporary = stage / 'camera-launch.tmp'
    try:
        os.symlink(CAMERA_LINK_TARGET, str(temporary))
        try:
            os.replace(str(temporary), str(launcher))
        except PermissionError:
            replace_privileged_component(root, temporary, launcher)
        sync_directory(launcher.parent)
    finally:
        if temporary.is_symlink():
            temporary.unlink()


def check_obsolete_media_metadata(root):
    directory = root / 'media-runtime-candidate'
    if directory.is_symlink() or (directory.exists() and not directory.is_dir()):
        raise ValueError('摄像头运行目录无效：' + str(directory))
    for name in OBSOLETE_MEDIA_METADATA:
        path = directory / name
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise ValueError('旧摄像头构建信息不是普通文件：' + str(path))


def remove_obsolete_media_metadata(root):
    check_obsolete_media_metadata(root)
    directory = root / 'media-runtime-candidate'
    changed = False
    for name in OBSOLETE_MEDIA_METADATA:
        path = directory / name
        if path.exists():
            try:
                path.unlink()
            except PermissionError:
                remove_privileged_file(root, path)
            changed = True
    if changed:
        sync_directory(directory)


def check_obsolete_debug_programs(root):
    directory = root / 'target/debug'
    if directory.is_symlink() or (directory.exists() and not directory.is_dir()):
        raise ValueError('旧程序目录无效：' + str(directory))
    for name in OBSOLETE_DEBUG_FILES:
        path = root / name
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise ValueError('旧程序不是普通文件：' + str(path))


def remove_obsolete_debug_programs(root):
    check_obsolete_debug_programs(root)
    directory = root / 'target/debug'
    changed = False
    for name in OBSOLETE_DEBUG_FILES:
        path = root / name
        if path.exists():
            try:
                path.unlink()
            except PermissionError:
                remove_privileged_file(root, path)
            changed = True
    if changed:
        sync_directory(directory)


def check_processes():
    for name in ('robotd', 'webpadd', 'padd', 'mediad'):
        result = subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            raise ValueError(name + ' 正在运行，请先按现有方式停止程序')
        if result.returncode != 1:
            raise ValueError('无法确认进程状态：' + name)
    for process in Path('/proc').glob('[0-9]*'):
        try:
            arguments = (process / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(argument.endswith(b'/console/server.py') for argument in arguments):
            raise ValueError('Studio 正在运行，请先按现有方式停止程序')


def ensure_robot_lock():
    """Create the standard lock after a reboot without replacing an existing inode."""
    try:
        descriptor = os.open(str(ROBOT_LOCK), os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
    except FileExistsError:
        return
    except PermissionError:
        code = ('import os,sys\n'
                'try: f=os.open(sys.argv[1],os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o644)\n'
                'except FileExistsError: pass\n'
                'else: os.close(f)\n')
        result = subprocess.run(['sudo', '-n', 'python3', '-c', code, str(ROBOT_LOCK)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        if result.returncode:
            raise ValueError('无法创建机器人锁文件，请先配置设备 sudo 权限')
    else:
        os.close(descriptor)


@contextlib.contextmanager
def exclusive_lock(path, create=False):
    flags = os.O_RDONLY
    if create:
        flags = os.O_RDWR | os.O_CREAT
    descriptor = os.open(str(path), flags | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    except BlockingIOError as error:
        raise ValueError('运行程序或其他安装操作正在占用：' + str(path)) from error
    finally:
        os.close(descriptor)


def smoke(root):
    for name in (ROBOTD, PADD, WEBPADD):
        result = subprocess.run([str(root / name), '--help'], stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, timeout=20)
        if result.returncode:
            raise ValueError('运行程序启动检查失败：' + name)


def git_blob(root, revision, path):
    result = subprocess.run(['git', '-C', str(root), 'show', '{}:{}'.format(revision, path.as_posix())],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    return result.stdout


def privileged_prefix():
    return [] if os.geteuid() == 0 else ['sudo', '-n']


def check_systemd_privilege():
    if not privileged_prefix():
        return
    result = subprocess.run(['sudo', '-n', 'true'], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    if result.returncode:
        raise ValueError('无法获取开机配置所需的 sudo 权限')


def reload_systemd():
    result = subprocess.run(privileged_prefix() + ['systemctl', 'daemon-reload'],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        detail = result.stderr.decode('utf-8', errors='replace').strip()
        raise ValueError('开机配置重载失败' + (': ' + detail if detail else ''))


def write_system_file(target, content, mode):
    if not content or len(content) > 64 * 1024:
        raise ValueError('开机配置文件内容无效')
    if mode not in (0o644, 0o755):
        raise ValueError('开机配置文件权限无效')
    code = (
        'import os,sys,tempfile\n'
        'from pathlib import Path\n'
        'target=Path(sys.argv[1]); parent=target.parent\n'
        'names=("source.conf","recording.conf","skip-mounted.conf","robot-bluetooth-auto-trust.py",'
        '"bluetooth_sixaxis.py","usbmount-if-needed","wonderduck-prompt-sounds.service",'
        '"10-help-text","00-header")\n'
        'if not target.is_absolute() or target.name not in names: sys.exit(2)\n'
        'if any(p.is_symlink() for p in target.parents): sys.exit(2)\n'
        'parent.mkdir(mode=0o755,parents=True,exist_ok=True)\n'
        'fd,name=tempfile.mkstemp(prefix="."+target.name+".",dir=str(parent))\n'
        'try:\n'
        ' with os.fdopen(fd,"wb") as stream:\n'
        '  stream.write(sys.stdin.buffer.read()); stream.flush(); os.fchmod(stream.fileno(),int(sys.argv[2],8)); os.fsync(stream.fileno())\n'
        ' os.replace(name,str(target))\n'
        ' fd=os.open(str(parent),os.O_RDONLY|os.O_DIRECTORY)\n'
        ' try: os.fsync(fd)\n'
        ' finally: os.close(fd)\n'
        'finally:\n'
        ' if os.path.exists(name): os.unlink(name)\n'
    )
    result = subprocess.run(privileged_prefix() + [sys.executable, '-c', code, str(target), format(mode, 'o')],
                            input=content, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode:
        detail = result.stderr.decode('utf-8', errors='replace').strip()
        raise ValueError('开机配置安装失败' + (': ' + detail if detail else ''))


def sync_system_files(files):
    for target, content, mode in files:
        write_system_file(target, content, mode)
    reload_systemd()


def sync_prompt_service():
    # The sound service is independent of robotd, so its low-battery warning
    # survives a robotd restart. Restart it to pick up the newly installed scripts.
    for action in (['enable'], ['restart']):
        result = subprocess.run(privileged_prefix() + ['systemctl'] + action + [PROMPT_SERVICE_TARGET.name],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
        if result.returncode:
            detail = result.stderr.decode('utf-8', errors='replace').strip()
            raise ValueError('提示音服务安装失败' + (': ' + detail if detail else ''))


def install(args):
    tag = 'v' + args.version
    channel = getattr(args, 'channel', 'main')
    if channel == 'main':
        version(args.version)
    elif channel != 'develop' or not development_release(args.version):
        raise ValueError('开发版格式为 0.1.0-dev.1.1')
    root = args.repo.resolve()
    if os.uname().machine != 'aarch64':
        raise ValueError('运行包仅支持 ARM64 设备')
    if root != SYSTEMD_ROOT:
        raise ValueError('开机配置只支持安装目录：' + str(SYSTEMD_ROOT))
    ensure_robot_lock()
    with exclusive_lock(root / '.runtime-install.lock', create=True):
        transactions = pending(root)
        if transactions:
            with exclusive_lock(ROBOT_LOCK):
                check_processes()
                for stage, state in transactions:
                    discard_pending(stage, root, state, args.version)
                    print('已清理上次中断的安装暂存文件')
        check_incomplete_version(root, args.version)
        main_revision, studio_revision = revisions(root)
        require_release_branch(root, main_revision, channel)
        check_camera_launcher(root)
        check_obsolete_media_metadata(root)
        check_obsolete_debug_programs(root)
        with exclusive_lock(ROBOT_LOCK):
            check_processes()
        stage = None
        state = {'schema': 2, 'status': 'preparing', 'version': args.version}
        try:
            stage = Path(tempfile.mkdtemp(prefix=TRANSACTION_PREFIX, dir=str(root)))
            write_journal(stage, state)
            source_base = release_download_base(root, args.base_url)
            base = source_base.rstrip('/') + '/' + quote(tag, safe='') + '/'
            manifest_path, signature_path = stage / MANIFEST_NAME, stage / SIGNATURE_NAME
            download(base + MANIFEST_NAME, manifest_path, 64 * 1024)
            download(base + SIGNATURE_NAME, signature_path, 4096)
            verify_signature(root / PUBLIC_KEY, manifest_path, signature_path)
            manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
            validate_manifest(manifest, tag)
            if (main_revision, studio_revision) != (manifest['source_revision'], manifest['studio_revision']):
                raise ValueError('源码版本与发布包不一致，请先检出发布标签并更新子模块')
            sources = ((BLUETOOTH_SIXAXIS_SOURCE, BLUETOOTH_SIXAXIS_TARGET, 0o644),
                       (BLUETOOTH_TRUST_SOURCE, BLUETOOTH_TRUST_TARGET, 0o755),
                       (USBMOUNT_GUARD_SOURCE, USBMOUNT_GUARD_TARGET, 0o755),
                       (USBMOUNT_DROPIN_SOURCE, USBMOUNT_DROPIN_TARGET, 0o644),
                       (SYSTEMD_SOURCE, SYSTEMD_DROPIN, 0o644),
                       (PADD_SOURCE, PADD_DROPIN, 0o644),
                       (WEBPADD_SOURCE, WEBPADD_DROPIN, 0o644),
                       (PROMPT_SERVICE_SOURCE, PROMPT_SERVICE_TARGET, 0o644),
                       (LOGIN_HEADER_SOURCE, LOGIN_HEADER_TARGET, 0o755),
                       (LOGIN_HELP_SOURCE, LOGIN_HELP_TARGET, 0o755))
            system_files = tuple((target, git_blob(root, manifest['source_revision'], source), mode)
                                 for source, target, mode in sources)
            if any(not content or len(content) > 64 * 1024 for _, content, _ in system_files):
                raise ValueError('开机配置文件内容无效')
            check_systemd_privilege()
            device = json.loads(DEVICE_VERSION.read_text(encoding='utf-8'))
            if version(device['hardware_version']) < version(manifest['min_hardware_version']):
                raise ValueError('当前硬件版本低于运行包要求')
            if version(device['image_version']) < version(manifest['min_image_version']):
                raise ValueError('当前镜像版本低于运行包要求')
            archive = stage / ARCHIVE_NAME
            print('正在下载运行包（{} 字节）'.format(manifest['size']), flush=True)
            download(base + ARCHIVE_NAME, archive, manifest['size'], manifest['size'])
            if digest_file(archive) != manifest['sha256']:
                raise ValueError('运行包 SHA-256 不匹配')
            unpack(archive, stage)
            fingerprints = {name: fingerprint(stage / 'payload' / name) for name in INSTALL_COMPONENTS}
            with exclusive_lock(ROBOT_LOCK):
                check_processes()
                check_incomplete_version(root, args.version)
                if revisions(root) != (manifest['source_revision'], manifest['studio_revision']):
                    raise ValueError('源码版本在安装期间发生变化')
                check_camera_launcher(root)
                check_obsolete_media_metadata(root)
                check_obsolete_debug_programs(root)
                state['status'] = 'applying'
                write_journal(stage, state)
                write_marker(root, stage, manifest, 'installing')
                installed = []
                for name in INSTALL_COMPONENTS:
                    if name == OBJECT_MODEL:
                        if install_missing_model(root, stage / 'payload' / name, root / name):
                            installed.append(name)
                    else:
                        replace_component(root, stage / 'payload' / name, root / name)
                        installed.append(name)
                for name in installed:
                    if fingerprint(root / name) != fingerprints[name]:
                        raise ValueError('安装后文件校验失败：' + name)
                remove_obsolete_media_metadata(root)
                smoke(root)
                sync_camera_launcher(root, stage)
                sync_system_files(system_files)
                sync_prompt_service()
                remove_obsolete_debug_programs(root)
                write_marker(root, stage, manifest, 'installed')
                state['status'] = 'committed'
                write_journal(stage, state)
            cleanup_stage(stage, root)
            print('运行包 {} 安装完成；请按现有方式启动机器人。'.format(args.version))
        except BaseException as error:
            if stage is not None and stage.exists():
                cleanup_stage(stage, root)
            if isinstance(error, Exception) and state['status'] == 'applying':
                raise ValueError('安装未完成，请重新安装版本 {}：{}'.format(args.version, error)) from error
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    make = commands.add_parser('package', help='打包并签名 ARM64 运行文件')
    make.add_argument('--version', required=True)
    make.add_argument('--repo', type=Path, default=ROOT)
    make.add_argument('--binaries', type=Path, required=True)
    make.add_argument('--web-build', type=Path, required=True)
    make.add_argument('--runtime-assets', type=Path, required=True)
    make.add_argument('--signing-key', type=Path, required=True)
    make.add_argument('--output', type=Path, required=True)
    make.add_argument('--min-hardware', default='1.0.0')
    make.add_argument('--min-image', default='1.0.0')
    apply = commands.add_parser('install', help='下载并安装指定发布版本')
    apply.add_argument('--version', required=True)
    apply.add_argument('--repo', type=Path, default=ROOT)
    apply.add_argument('--base-url', help='Override the release download base URL')
    apply.add_argument('--channel', choices=('main', 'develop'), default='main')
    args = parser.parse_args()
    try:
        (package if args.command == 'package' else install)(args)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError, tarfile.TarError) as error:
        print('运行包操作失败：' + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
