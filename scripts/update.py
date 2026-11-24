#!/usr/bin/env python3
"""Interactively install an official WonderDuck release."""

import argparse
from contextlib import contextmanager
import fcntl
from html.parser import HTMLParser
import importlib.util
import inspect
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


ROOT = Path('/home/wonderduck/wonderduck')
STATUS_FILE = '.software-update-status.json'
RECOVERY_FILE = '.software-update-recovery.json'
RECOVERY_HELPER = '.software-update-recovery'
UPDATE_LOCK = '.software-update.lock'
ROBOT_SOCKET = '/run/robotd.sock'
RECOVERY_HOOK = (
    "ExecStartPre=+/bin/sh -c 'if test -f /home/wonderduck/wonderduck/.software-update-recovery.json; "
    "then exec /usr/bin/sudo -n -u wonderduck -- /usr/bin/python3 -B "
    "/home/wonderduck/wonderduck/.software-update-recovery/update.py --recover-at-start; fi'"
)
VERSION_NUMBER = r'(0|[1-9][0-9]*)'
VERSION = r'\.'.join([VERSION_NUMBER] * 3)
VERSION_PATTERNS = {
    'main': re.compile(VERSION),
    'develop': re.compile(VERSION + r'-dev\.' + VERSION_NUMBER + r'\.' + VERSION_NUMBER),
}
SOURCES = {
    '1': ('GitHub', 'https://github.com/Hiwonder/wonderduck.git',
          'https://api.github.com/repos/Hiwonder/wonderduck/releases/latest',
          'https://github.com/Hiwonder/wonderduck/releases/download'),
    '2': ('Gitee', 'https://gitee.com/hiwonder_official/wonderduck.git',
          'https://gitee.com/api/v5/repos/hiwonder_official/wonderduck/releases/latest',
          'https://gitee.com/hiwonder_official/wonderduck/releases/download'),
}
TEXT = {
    'zh': {
        'source_prompt': '选择更新来源 [1 GitHub / 2 Gitee，默认 1]：',
        'source_invalid': '请输入 1 或 2。',
        'source_selected': '更新来源：{}',
        'version_prompt': '输入版本号（例如 0.1.4，直接回车使用最新版本）：',
        'version_invalid': '版本号格式不正确，请输入例如 0.1.4。',
        'latest_search': '正在获取最新正式版本……',
        'dev_version_prompt': '输入开发版本号（例如 0.1.0-dev.1.2，直接回车使用最新开发版）：',
        'dev_version_invalid': '开发版本号格式不正确，请输入例如 0.1.0-dev.1.2。',
        'dev_latest_search': '正在获取最新开发版本……',
        'version_selected': '更新版本：v{}',
        'release_unavailable': '无法获取该版本的完整运行包，请检查网络或输入其他版本。',
        'no_repo': '未找到设备程序目录，请确认已烧录并配置 WonderDuck 镜像。',
        'dirty': '{} 存在未提交的修改，请先处理后再更新。',
        'fetch': '正在获取源码……',
        'reload': '正在刷新服务配置……',
        'stop': '正在停止机器人服务……',
        'install': '正在安装更新，请稍候……',
        'start': '正在启动机器人服务……',
        'done': '更新完成。',
        'failed': '更新失败，请检查上面的错误后重试。',
        'stopped': '机器人服务保持停止；请处理问题后重新运行更新命令。',
        'cancelled': '更新已取消。',
    },
    'en': {
        'source_prompt': 'Update source [1 GitHub / 2 Gitee, default 1]: ',
        'source_invalid': 'Enter 1 or 2.',
        'source_selected': 'Update source: {}',
        'version_prompt': 'Version (for example 0.1.4; press Enter for the latest): ',
        'version_invalid': 'Invalid version. Enter a version such as 0.1.4.',
        'latest_search': 'Finding the latest stable release...',
        'dev_version_prompt': 'Development version (for example 0.1.0-dev.1.2; press Enter for the latest): ',
        'dev_version_invalid': 'Invalid development version. Enter a version such as 0.1.0-dev.1.2.',
        'dev_latest_search': 'Finding the latest development release...',
        'version_selected': 'Update version: v{}',
        'release_unavailable': 'The complete release could not be found. Check the network or enter another version.',
        'no_repo': 'The device software directory was not found. Use a configured WonderDuck image.',
        'dirty': '{} has uncommitted changes. Resolve them before updating.',
        'fetch': 'Fetching source...',
        'reload': 'Reloading service configuration...',
        'stop': 'Stopping robot service...',
        'install': 'Installing the update...',
        'start': 'Starting robot service...',
        'done': 'Update complete.',
        'failed': 'Update failed. Check the error above and try again.',
        'stopped': 'The robot service remains stopped. Resolve the problem and rerun the update.',
        'cancelled': 'Update cancelled.',
    },
}


def parse_version(value, channel):
    if not isinstance(value, str):
        return None
    value = value.strip()
    if value.startswith('v'):
        value = value[1:]
    return value if VERSION_PATTERNS[channel].fullmatch(value) else None


def read_url(url, limit, timeout=15):
    request = urllib.request.Request(url, headers={'User-Agent': 'WonderDuck-Updater', 'Accept': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        data = response.read(limit + 1)
    if not data or len(data) > limit:
        raise ValueError('invalid release response')
    return data


def check_release(source, version):
    base = source[3] + '/v' + version + '/'
    manifest = json.loads(read_url(base + 'manifest.json', 64 * 1024))
    if (not isinstance(manifest, dict) or manifest.get('tag') != 'v' + version
            or manifest.get('version') != version or manifest.get('architecture') != 'aarch64'
            or manifest.get('archive') != 'aarch64.tar.gz'):
        raise ValueError('invalid release manifest')
    read_url(base + 'manifest.json.sig', 4096)


def latest_release(source, channel):
    if channel == 'main':
        try:
            release = json.loads(read_url(source[2], 256 * 1024))
            version = parse_version(release.get('tag_name'), channel) if isinstance(release, dict) else None
            if version and not release.get('draft') and not release.get('prerelease'):
                check_release(source, version)
                return version
        except (OSError, ValueError, TypeError, urllib.error.URLError):
            pass
    # The stable-release endpoint omits prereleases. Tag discovery also works
    # when the stable-release API is rate-limited.
    env = dict(os.environ, GIT_TERMINAL_PROMPT='0')
    result = subprocess.run(['git', 'ls-remote', '--tags', '--refs', source[1]], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, timeout=30, check=True)
    versions = set()
    for line in result.stdout.splitlines():
        ref = line.split()[-1] if line.split() else ''
        if ref.startswith('refs/tags/v'):
            candidate = parse_version(ref[len('refs/tags/'):], channel)
            if candidate:
                versions.add(candidate)
    pattern = VERSION_PATTERNS[channel]
    for version in sorted(versions, key=lambda item: tuple(map(int, pattern.fullmatch(item).groups())),
                          reverse=True):
        try:
            check_release(source, version)
            return version
        except (OSError, ValueError, TypeError, urllib.error.URLError):
            continue
    raise ValueError('no complete release for ' + channel)


def latest_stable_for_browser(source):
    # The release list includes stable tags even when Gitee's "latest" is a
    # development build. Inspect the asset listing without fetching a bundle.
    endpoint = source[2].rsplit('/latest', 1)[0]
    releases = json.loads(read_url(endpoint + '?per_page=100', 4 * 1024 * 1024))
    if not isinstance(releases, list):
        raise ValueError('发布列表格式无效')
    versions = []
    for release in releases:
        if release.get('draft') or release.get('prerelease'):
            continue
        version = parse_version(release.get('tag_name'), 'main')
        assets = {asset.get('name') for asset in release.get('assets', ())}
        if version and {'manifest.json', 'manifest.json.sig', 'aarch64.tar.gz'} <= assets:
            versions.append(version)
    if not versions:
        raise ValueError('找不到完整的软件发布包')
    return max(versions, key=lambda version: tuple(map(int, version.split('.'))))


def command(args, **kwargs):
    env = dict(os.environ, GIT_TERMINAL_PROMPT='0')
    kwargs.setdefault('stderr', subprocess.PIPE)
    return subprocess.run(args, cwd=ROOT, env=env, check=True, **kwargs)


def load_installer(source=None):
    spec = importlib.util.spec_from_file_location('runtime_release', source or ROOT / 'scripts/runtime_release.py')
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    return runtime


def prepare_release(runtime, source, version, directory, report):
    tag = 'v' + version
    base = source[3] + '/' + tag + '/'
    release = directory / tag
    release.mkdir()
    manifest, signature = (release / name for name in ('manifest.json', 'manifest.json.sig'))
    manifest.write_bytes(read_url(base + manifest.name, 64 * 1024))
    signature.write_bytes(read_url(base + signature.name, 4096))
    runtime.verify_signature(ROOT / runtime.PUBLIC_KEY, manifest, signature)
    data = json.loads(manifest.read_text(encoding='utf-8'))
    runtime.validate_manifest(data, tag)
    device = json.loads(runtime.DEVICE_VERSION.read_text(encoding='utf-8'))
    for key in ('hardware_version', 'image_version'):
        if runtime.version(device[key]) < runtime.version(data['min_' + key]):
            raise ValueError('设备版本低于运行包要求')
    archive = release / data['archive']
    report('download', progress=0)
    options = {}
    if 'progress_callback' in inspect.signature(runtime.download).parameters:
        options['progress_callback'] = lambda received, size: report('download', progress=received * 100 // size)
    runtime.download(base + archive.name, archive, data['size'], data['size'], progress=True, **options)
    report('verify')
    if runtime.digest_file(archive) != data['sha256']:
        raise ValueError('运行包 SHA-256 不匹配')
    return data


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_update_json(path, data, *, durable=False):
    pending = tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False)
    temporary = Path(pending.name)
    try:
        with pending as stream:
            json.dump(data, stream)
            stream.flush()
            if durable:
                os.fsync(stream.fileno())
        os.replace(temporary, path)
        if durable:
            sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def report_status(version, phase, **details):
    write_update_json(ROOT / STATUS_FILE, dict(version=version, phase=phase, **details))


def error_text(error):
    if isinstance(error, subprocess.CalledProcessError) and error.stderr:
        return error.stderr.decode('utf-8', errors='replace').strip()[-1500:]
    return str(error)


def git_setting(repo, *args):
    result = subprocess.run(['git', '-C', str(repo), *args], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, timeout=10)
    if result.returncode not in (0, 1) or (result.returncode == 1 and result.stderr):
        raise ValueError(result.stderr.strip() or '无法读取源码状态')
    return result.stdout.strip()


def checkpoint_paths(runtime):
    paths = [ROOT / name for name in runtime.INSTALL_COMPONENTS if name != runtime.OBJECT_MODEL]
    model = ROOT / runtime.OBJECT_MODEL
    if not runtime.exists(model):
        paths.append(model)
    paths.extend(ROOT / name for name in (*runtime.OBSOLETE_DEBUG_FILES,
                                          *runtime.PRIVILEGED_OBSOLETE_FILES,
                                          runtime.CAMERA_LAUNCHER, runtime.MARKER))
    paths.extend(getattr(runtime, name) for name in (
        'SYSTEMD_DROPIN', 'PADD_DROPIN', 'WEBPADD_DROPIN', 'PROMPT_SERVICE_TARGET',
        'LOGIN_HELP_TARGET', 'LOGIN_HEADER_TARGET', 'BLUETOOTH_TRUST_TARGET',
        'BLUETOOTH_SIXAXIS_TARGET', 'USBMOUNT_GUARD_TARGET', 'USBMOUNT_DROPIN_TARGET'))
    paths.append(Path('/etc/systemd/system/multi-user.target.wants') / runtime.PROMPT_SERVICE_TARGET.name)
    return list(dict.fromkeys(paths))


def validate_checkpoint_path(runtime, path):
    if any(parent.is_symlink() for parent in path.parents):
        raise ValueError('回退路径包含符号链接：' + str(path))
    if ROOT in path.parents:
        runtime.check_mounted_directory(ROOT, path)
    elif path.is_dir():
        raise ValueError('开机配置路径应为文件：' + str(path))


def create_checkpoint(runtime, archive):
    studio = ROOT / 'extensions/wonderduck_studio'
    worktrees = []
    for repo in (ROOT, studio):
        revision = git_setting(repo, 'rev-parse', '--verify', 'HEAD^{commit}')
        if not re.fullmatch(r'[0-9a-f]{40}', revision):
            raise ValueError('无法读取更新前的源码版本：' + str(repo))
        worktrees.append((repo, revision, git_setting(repo, 'symbolic-ref', '--quiet', '--short', 'HEAD'),
                          git_setting(repo, 'config', '--local', '--get', 'remote.origin.url')))
    submodule_url = git_setting(ROOT, 'config', '--local', '--get',
                                'submodule.extensions/wonderduck_studio.url')
    paths = checkpoint_paths(runtime)
    for path in paths:
        validate_checkpoint_path(runtime, path)
    present = [str(path.relative_to('/')) for path in paths if runtime.exists(path)]
    with archive.open('wb') as stream:
        command(['sudo', '-n', 'tar', '-C', '/', '-cpf', '-', '--', *present], stdout=stream, timeout=180)
        stream.flush()
        os.fsync(stream.fileno())
    sync_directory(archive.parent)
    return worktrees, submodule_url, paths


def restore_git_setting(repo, name, value):
    if value:
        command(['git', '-C', str(repo), 'config', '--local', name, value], timeout=10)
    elif git_setting(repo, 'config', '--local', '--get', name):
        command(['git', '-C', str(repo), 'config', '--local', '--unset', name], timeout=10)


def restore_checkpoint(runtime, archive, checkpoint, restore_runtime, *, boot=False):
    worktrees, submodule_url, paths = checkpoint
    for repo, revision, branch, origin in worktrees:
        checkout = [branch] if branch else ['--detach', revision]
        if git_setting(repo, 'rev-parse', 'HEAD') != revision or git_setting(repo, 'symbolic-ref', '--quiet', '--short', 'HEAD') != branch:
            command(['git', '-C', str(repo), 'checkout', '--no-overwrite-ignore', *checkout], timeout=30)
        if git_setting(repo, 'rev-parse', 'HEAD') != revision:
            raise ValueError('未能恢复更新前的源码版本：' + str(repo))
        restore_git_setting(repo, 'remote.origin.url', origin)
    restore_git_setting(ROOT, 'submodule.extensions/wonderduck_studio.url', submodule_url)
    if not restore_runtime:
        return
    for path in paths:
        validate_checkpoint_path(runtime, path)
    command(['sudo', '-n', 'rm', '-rf', '--one-file-system', '--', *map(str, paths)], timeout=120)
    with archive.open('rb') as stream:
        command(['sudo', '-n', 'tar', '-C', '/', '--numeric-owner', '-xpf', '-'], stdin=stream, timeout=180)
    command(['sudo', '-n', 'systemctl', 'daemon-reload'], timeout=20)
    if boot:
        return
    if runtime.PROMPT_SERVICE_TARGET.is_file():
        command(['sudo', '-n', 'systemctl', 'restart', runtime.PROMPT_SERVICE_TARGET.name], timeout=30)
    else:
        command(['sudo', '-n', 'systemctl', 'stop', runtime.PROMPT_SERVICE_TARGET.name], timeout=30)


def record_recovery(stage, version, checkpoint, job_id, *, phase='stopping'):
    worktrees, submodule_url, paths = checkpoint
    record = dict(stage=stage.name, version=version, job_id=job_id, phase=phase,
                  worktrees=[(str(repo), revision, branch, origin) for repo, revision, branch, origin in worktrees],
                  submodule_url=submodule_url, paths=list(map(str, paths)))
    write_update_json(ROOT / RECOVERY_FILE, record, durable=True)


def prepare_recovery_helper(runtime, revision):
    helper = ROOT / RECOVERY_HELPER
    helper.mkdir(mode=0o700, exist_ok=True)
    source = Path(__file__).resolve()
    update = source.read_bytes() if source.is_file() else runtime.git_blob(ROOT, revision, Path('scripts/update.py'))
    if b'--recover-at-start' not in update:
        raise ValueError('发布版本缺少软件更新恢复程序')
    for name, content in (('update.py', update),
                          ('runtime_release.py', (ROOT / 'scripts/runtime_release.py').read_bytes())):
        temporary = helper / (name + '.pending')
        with temporary.open('wb') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, helper / name)
    sync_directory(helper)
    sync_directory(ROOT)


def ensure_recovery_hook(runtime):
    path = runtime.SYSTEMD_DROPIN.with_name('wonderduck-update-recovery.conf')
    contents = ('[Service]\n' + RECOVERY_HOOK + '\n').encode()
    if path.is_file() and path.read_bytes() == contents:
        return
    with tempfile.NamedTemporaryFile() as stream:
        stream.write(contents)
        stream.flush()
        pending = path.with_suffix('.pending')
        command(['sudo', '-n', '/usr/bin/install', '-m', '0644', stream.name, str(pending)], timeout=10)
        command(['sudo', '-n', '/usr/bin/mv', '--', str(pending), str(path)], timeout=10)
        command(['sudo', '-n', '/usr/bin/sync', '-f', str(path)], timeout=10)
    runtime.reload_systemd()


def clear_recovery():
    (ROOT / RECOVERY_FILE).unlink(missing_ok=True)
    sync_directory(ROOT)
    helper = ROOT / RECOVERY_HELPER
    try:
        if helper.is_dir():
            for name in ('update.py', 'runtime_release.py'):
                (helper / name).unlink(missing_ok=True)
            helper.rmdir()
    except OSError as error:
        print(error_text(error), file=sys.stderr)


def recovery_checkpoint(runtime, record):
    stage_name = record['stage']
    if record['phase'] not in ('stopping', 'applying', 'starting'):
        raise ValueError('软件更新恢复阶段无效')
    if not isinstance(stage_name, str) or not re.fullmatch(r'\.software-update-[A-Za-z0-9_]+', stage_name):
        raise ValueError('软件更新恢复目录无效')
    stage = ROOT / stage_name
    archive = stage / 'previous.tar'
    if stage.is_symlink() or archive.is_symlink() or not archive.is_file():
        raise ValueError('没有找到软件更新恢复文件')
    worktrees = [(Path(repo), revision, branch, origin) for repo, revision, branch, origin in record['worktrees']]
    if [item[0] for item in worktrees] != [ROOT, ROOT / 'extensions/wonderduck_studio']:
        raise ValueError('软件更新恢复仓库无效')
    if any(not re.fullmatch(r'[0-9a-f]{40}', revision) for _, revision, _, _ in worktrees):
        raise ValueError('软件更新恢复版本无效')
    allowed = set(checkpoint_paths(runtime)) | {ROOT / runtime.OBJECT_MODEL}
    paths = list(map(Path, record['paths']))
    if not paths or not set(paths) <= allowed:
        raise ValueError('软件更新恢复文件清单无效')
    return stage, archive, (worktrees, record['submodule_url'], paths)


def recover_at_start():
    journal = ROOT / RECOVERY_FILE
    if not journal.exists():
        return
    descriptor = os.open(ROOT / UPDATE_LOCK, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            # Only the live updater may start the newly installed service.
            if journal.exists() and json.loads(journal.read_text(encoding='utf-8')).get('phase') != 'starting':
                raise RuntimeError('软件更新正在安装，暂不能启动机器人')
            return
        if not journal.exists():
            return
        runtime = load_installer(ROOT / RECOVERY_HELPER / 'runtime_release.py')
        record = json.loads(journal.read_text(encoding='utf-8'))
        stage, archive, checkpoint = recovery_checkpoint(runtime, record)
        details = dict(job_id=record['job_id'], failed_phase=record['phase'],
                       error='软件更新在完成前中断')
        report_status(record['version'], 'rollback', **details)
        try:
            runtime.ensure_robot_lock()
            with runtime.exclusive_lock(runtime.ROBOT_LOCK):
                runtime.check_processes()
                restore_checkpoint(runtime, archive, checkpoint, record['phase'] != 'stopping', boot=True)
                for directory, transaction in runtime.pending(ROOT):
                    if transaction.get('version') == record['version']:
                        runtime.discard_pending(directory, ROOT, transaction, record['version'])
        except Exception as error:
            report_status(record['version'], 'failed', rollback='failed', rollback_error=error_text(error), **details)
            raise
        report_status(record['version'], 'failed', rollback='done', **details)
        clear_recovery()
        shutil.rmtree(stage)
    finally:
        os.close(descriptor)


@contextmanager
def installation_lease(runtime, socket_path):
    state = command(['systemctl', 'show', 'robotd', '-p', 'ActiveState', '--value'],
                    stdout=subprocess.PIPE, timeout=10).stdout.strip()
    if state in (b'inactive', b'failed'):
        runtime.ensure_robot_lock()
        with runtime.exclusive_lock(runtime.ROBOT_LOCK):
            runtime.check_processes()
            yield
        return
    sys.path.insert(0, str(ROOT / 'extensions/wonderduck_studio/console'))
    try:
        from app.robot.studio import MaintenanceLease
        from app.robot.workspace_rest import ensure_rest, rest_status
    finally:
        sys.path.pop(0)

    ensure_rest(socket_path)
    lease = MaintenanceLease(socket_path)
    try:
        lease.request('acquire')
        # Start may have arrived before we took the bus. Check again once no
        # controller can change the state, then hold the lease through stop.
        if rest_status(socket_path) != ('rested', False) or not lease.alive():
            raise RuntimeError('安装前无法确认机器人已休息并锁定控制')
        yield
    finally:
        lease.close()


class StudioAssets(HTMLParser):
    def __init__(self):
        super().__init__()
        self.paths = set()
        self.has_script = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'script':
            path = attrs.get('src')
        elif tag == 'link' and attrs.get('rel') in ('stylesheet', 'modulepreload'):
            path = attrs.get('href')
        else:
            return
        if isinstance(path, str) and re.fullmatch(r'/assets/[A-Za-z0-9_.-]+\.(js|css)', path):
            self.paths.add(path)
            self.has_script |= tag == 'script'


def check_studio(port, *, allow_legacy=False):
    base = f'http://127.0.0.1:{port}'
    if not json.loads(read_url(base + '/api/info', 256 * 1024, timeout=2)).get('studio'):
        raise ValueError('Studio 服务尚未就绪')
    paths = set()
    for page in ('/', '/debug.html'):
        assets = StudioAssets()
        assets.feed(read_url(base + page, 256 * 1024, timeout=2).decode('utf-8'))
        if not assets.has_script:
            raise ValueError('Studio 页面缺少构建资源')
        paths.update(assets.paths)
    for path in paths:
        with urllib.request.urlopen(base + path, timeout=2) as response:
            expected = ('text/javascript', 'application/javascript') if path.endswith('.js') else ('text/css',)
            if response.headers.get_content_type() not in expected or not response.read(1):
                raise ValueError('Studio 静态资源无效')
    try:
        with urllib.request.urlopen('http://127.0.0.1:8081/health', timeout=2) as response:
            if json.loads(response.read(4096)).get('ready') is not True:
                raise ValueError('网页操控服务尚未就绪')
    except urllib.error.HTTPError as error:
        if error.code != 404 or not allow_legacy:
            raise
        with urllib.request.urlopen('http://127.0.0.1:8081/controls.js', timeout=2) as response:
            if not response.read(1):
                raise ValueError('网页操控服务尚未就绪')


def wait_robot_ready(expected_revision=None, studio_port=8090, socket_path=ROBOT_SOCKET):
    deadline = time.monotonic() + 40
    reason = '没有收到机器人健康状态'
    while time.monotonic() < deadline:
        try:
            if expected_revision:
                identity = json.loads(Path('/run/robotd/identity.json').read_text(encoding='utf-8'))
                if identity.get('revision') != expected_revision:
                    raise ValueError('运行程序与发布版本不一致')
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(2)
                connection.connect(str(socket_path))
                connection.sendall(b'{"jsonrpc":"2.0","id":1,"method":"robot.health"}\n')
                with connection.makefile('rb') as stream:
                    response = json.loads(stream.readline(65536))
            health = response.get('result') or {}
            if health.get('healthy') is True:
                check_studio(studio_port, allow_legacy=expected_revision is None)
                return
            reason = health.get('reason') or '机器人健康检查失败'
        except (OSError, ValueError) as error:
            reason = str(error)
        time.sleep(1)
    raise RuntimeError('等待机器人启动超时：' + reason)


def check_worktrees(text):
    for path, label in ((ROOT, 'WonderDuck'), (ROOT / 'extensions/wonderduck_studio', 'WonderDuck Studio')):
        if path != ROOT and not (path / '.git').exists():
            continue
        status = command(['git', '-C', str(path), 'status', '--porcelain=v1',
                          '--untracked-files=no', '--ignore-submodules=all'], stdout=subprocess.PIPE).stdout
        if status.strip():
            raise ValueError(text['dirty'].format(label))


def install(source, version, text, channel, unattended=False, job_id=None, studio_port=8090,
            socket_path=ROBOT_SOCKET):
    try:
        runtime = load_installer()
        with runtime.exclusive_lock(ROOT / UPDATE_LOCK, create=True):
            if (ROOT / RECOVERY_FILE).exists():
                raise ValueError('上次软件更新尚未恢复，请启动机器人服务以完成恢复')
            return install_release(runtime, source, version, text, channel, unattended, job_id, studio_port, socket_path)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(error_text(error), file=sys.stderr)
        return 1


def install_release(runtime, source, version, text, channel, unattended, job_id=None, studio_port=8090,
                    socket_path=ROBOT_SOCKET):
    channel_args = ['--channel', channel] if channel == 'develop' else []
    phase = 'prepare'

    def report(next_phase, **details):
        nonlocal phase
        phase = next_phase
        report_status(version, phase, job_id=job_id, **details)

    checkpoint = None
    stage = None
    keep_checkpoint = False
    service_touched = False
    runtime_touched = False
    changed_origin = False
    existing_transactions = set()
    try:
        report(phase)
        check_worktrees(text)
        command(['sudo', '-n', 'true'] if unattended else ['sudo', '-v'], timeout=30)
        stage = Path(tempfile.mkdtemp(prefix='.software-update-', dir=str(ROOT)))
        phase = 'download'
        release = prepare_release(runtime, source, version, stage, report)
        phase = 'checkpoint'
        report(phase)
        archive = stage / 'previous.tar'
        checkpoint = create_checkpoint(runtime, archive)
        phase = 'fetch'
        report(phase)
        print(text['fetch'], flush=True)
        changed_origin = True
        command(['git', 'remote', 'set-url', 'origin', source[1]], timeout=10)
        command(['git', 'fetch', '--no-tags', '--no-recurse-submodules', 'origin',
                 '+refs/heads/{0}:refs/remotes/origin/{0}'.format(channel)], timeout=90)
        command(['git', 'fetch', '--no-tags', '--no-recurse-submodules', 'origin', 'refs/tags/v' + version], timeout=90)
        command(['git', 'merge-base', '--is-ancestor', 'FETCH_HEAD', 'refs/remotes/origin/' + channel], timeout=10)
        revision = git_setting(ROOT, 'rev-parse', 'FETCH_HEAD^{commit}')
        studio_revision = git_setting(ROOT, 'rev-parse', revision + ':extensions/wonderduck_studio')
        if (revision, studio_revision) != (release['source_revision'], release['studio_revision']):
            raise ValueError('发布标签与签名清单的源码版本不一致')
        source_script = command(['git', 'show', 'FETCH_HEAD:scripts/update-source.py'],
                                stdout=subprocess.PIPE, timeout=10).stdout
        if not source_script:
            raise ValueError('empty source updater')
        check_worktrees(text)
        reload_needed = command(['systemctl', 'show', 'robotd', '-p', 'NeedDaemonReload', '--value'],
                                stdout=subprocess.PIPE, timeout=10).stdout.strip()
        if reload_needed == b'yes':
            print(text['reload'], flush=True)
            command(['sudo', '-n', 'systemctl', 'daemon-reload'], timeout=20)
        prepare_recovery_helper(runtime, revision)
        ensure_recovery_hook(runtime)
        report('rest')
        with installation_lease(runtime, socket_path):
            report('install')
            record_recovery(stage, version, checkpoint, job_id)
            print(text['stop'], flush=True)
            service_touched = True
            command(['sudo', '-n', 'systemctl', 'stop', 'robotd'], timeout=35)
        command([sys.executable, '-', version, *channel_args], input=source_script, timeout=180)
        print(text['install'], flush=True)
        existing_transactions = {directory for directory, _ in runtime.pending(ROOT)}
        record_recovery(stage, version, checkpoint, job_id, phase='applying')
        runtime_touched = True
        command([sys.executable, 'scripts/runtime_release.py', 'install', '--version', version,
                 '--base-url', stage.as_uri(), *channel_args], timeout=480)
        phase = 'restart'
        report(phase)
        record_recovery(stage, version, checkpoint, job_id, phase='starting')
        print(text['start'], flush=True)
        command(['sudo', '-n', 'systemctl', 'start', 'robotd'], timeout=35)
        wait_robot_ready(revision, studio_port, socket_path)
        clear_recovery()
        try:
            report('done')
        except OSError as error:
            print(error_text(error), file=sys.stderr)
        print(text['done'], flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        cause = error_text(error) or text['cancelled']
        result = {'failed_phase': phase, 'error': cause, 'rollback': 'unneeded'}
        if checkpoint and changed_origin:
            try:
                report('rollback', **result)
                if service_touched:
                    with installation_lease(runtime, socket_path):
                        record_recovery(stage, version, checkpoint, job_id,
                                        phase='applying' if runtime_touched else 'stopping')
                        command(['sudo', '-n', 'systemctl', 'stop', 'robotd'], timeout=35)
                if service_touched:
                    with runtime.exclusive_lock(runtime.ROBOT_LOCK):
                        runtime.check_processes()
                        restore_checkpoint(runtime, archive, checkpoint, runtime_touched)
                        if runtime_touched:
                            for directory, record in runtime.pending(ROOT):
                                if directory not in existing_transactions and record.get('version') == version:
                                    runtime.discard_pending(directory, ROOT, record, version)
                else:
                    restore_checkpoint(runtime, archive, checkpoint, False)
                if service_touched:
                    record_recovery(stage, version, checkpoint, job_id, phase='starting')
                    command(['sudo', '-n', 'systemctl', 'start', 'robotd'], timeout=35)
                    wait_robot_ready(studio_port=studio_port, socket_path=socket_path)
                    clear_recovery()
                result['rollback'] = 'done'
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as rollback_error:
                keep_checkpoint = True
                result.update(rollback='failed', rollback_error=error_text(rollback_error), checkpoint=str(stage))
        report('failed', **result)
        print(cause, file=sys.stderr)
        if result['rollback'] == 'done':
            print('已恢复更新前的软件。' if text is TEXT['zh'] else 'The previous software has been restored.', file=sys.stderr)
        elif keep_checkpoint:
            print(result['rollback_error'], file=sys.stderr)
            print(str(stage), file=sys.stderr)
        print(text['failed'], file=sys.stderr)
        return 130 if isinstance(error, KeyboardInterrupt) else 1
    finally:
        if stage is not None and not keep_checkpoint:
            shutil.rmtree(stage)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lang', choices=('zh', 'en'), default='zh')
    parser.add_argument('--channel', choices=('main', 'develop'), default='main', help=argparse.SUPPRESS)
    parser.add_argument('--source', choices=('github', 'gitee'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--version')
    mode.add_argument('--preflight', action='store_true')
    mode.add_argument('--recover-at-start', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--job-id', help=argparse.SUPPRESS)
    parser.add_argument('--studio-port', type=int, default=8090, help=argparse.SUPPRESS)
    parser.add_argument('--robot-socket', default=ROBOT_SOCKET, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.recover_at_start:
        try:
            recover_at_start()
            return 0
        except (OSError, KeyError, TypeError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            print(error_text(error), file=sys.stderr)
            return 1
    text = TEXT[args.lang]
    prefix = 'dev_' if args.channel == 'develop' else ''
    if not (ROOT / '.git').exists():
        print(text['no_repo'], file=sys.stderr)
        return 1
    if args.check or args.version or args.preflight:
        if not args.source:
            parser.error('--source is required for non-interactive updates')
        source = SOURCES['1' if args.source == 'github' else '2']
        try:
            if args.check:
                version = latest_stable_for_browser(source) if args.channel == 'main' else latest_release(source, args.channel)
                print(json.dumps({'version': version}))
                return 0
            if args.preflight:
                check_worktrees(text)
                return 0
            version = parse_version(args.version, args.channel)
            if not version:
                raise ValueError(text[prefix + 'version_invalid'])
            if args.job_id and not re.fullmatch(r'[0-9a-f]{16}', args.job_id):
                raise ValueError('更新任务编号无效')
            if args.studio_port is not None and not 1 <= args.studio_port <= 65535:
                raise ValueError('Studio 端口无效')
            return install(source, version, text, args.channel, unattended=True,
                           job_id=args.job_id, studio_port=args.studio_port, socket_path=args.robot_socket)
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            print(str(error), file=sys.stderr)
            return 1
    try:
        while True:
            choice = input(text['source_prompt']).strip().lower() or '1'
            choice = {'github': '1', 'gitee': '2'}.get(choice, choice)
            if choice in SOURCES:
                break
            print(text['source_invalid'])
        source = SOURCES[choice]
        print(text['source_selected'].format(source[0]), flush=True)
        while True:
            entered = input(text[prefix + 'version_prompt']).strip()
            version = parse_version(entered, args.channel) if entered else None
            if entered and not version:
                print(text[prefix + 'version_invalid'])
                continue
            try:
                if not entered:
                    print(text[prefix + 'latest_search'], flush=True)
                    version = latest_release(source, args.channel)
                else:
                    check_release(source, version)
            except (OSError, ValueError, TypeError, urllib.error.URLError, subprocess.SubprocessError):
                print(text['release_unavailable'])
                continue
            print(text['version_selected'].format(version), flush=True)
            return install(source, version, text, args.channel)
    except (EOFError, KeyboardInterrupt):
        print('\n' + text['cancelled'], file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
