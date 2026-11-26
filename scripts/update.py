#!/usr/bin/env python3
"""Interactively install an official WonderDuck release."""

import argparse
from contextlib import contextmanager, redirect_stdout
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
        'interrupted': '软件更新在完成前中断',
        'restored': '已恢复更新前的软件。',
        'download_progress': '下载进度：',
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
        'interrupted': 'The software update was interrupted before completion.',
        'restored': 'The previous software has been restored.',
        'download_progress': 'Download progress: ',
    },
}

# The terminal bootstrap also runs with older installed and signed release
# scripts. Translate their diagnostics here so both CLI and Studio receive them.
UPDATE_DIAGNOSTICS = {
    '发布响应无效': 'Invalid release response',
    '发布清单无效': 'Invalid release manifest',
    '发布版本缺少源码更新程序': 'The release is missing the source update script',
    '发布列表格式无效': 'Invalid release list',
    '找不到完整的软件发布包': 'No complete software release found',
    '设备版本低于运行包要求': 'This hardware or system image does not meet the release requirements',
    '无法读取源码状态': 'Could not read the source checkout status',
    '回退路径包含符号链接': 'A rollback path contains a symbolic link',
    '开机配置路径应为文件': 'A startup configuration path must be a file',
    '无法读取更新前的源码版本': 'Could not read the source revision before the update',
    '未能恢复更新前的源码版本': 'Could not restore the previous source revision',
    '发布版本缺少软件更新恢复程序': 'The release is missing the update recovery script',
    '软件更新恢复阶段无效': 'Invalid update recovery stage',
    '软件更新恢复目录无效': 'Invalid update recovery directory',
    '没有找到软件更新恢复文件': 'Update recovery files were not found',
    '软件更新恢复仓库无效': 'Invalid repository in the update recovery record',
    '软件更新恢复版本无效': 'Invalid revision in the update recovery record',
    '软件更新恢复文件清单无效': 'Invalid file list in the update recovery record',
    '软件更新正在安装，暂不能启动机器人': 'Wait for the software installation to finish before starting the robot',
    '安装前无法确认机器人已休息并锁定控制': 'Could not confirm that the robot was at rest and lock the controls before installation',
    'Studio 服务尚未就绪': 'Studio is not ready',
    'Studio 页面缺少构建资源': 'Studio is missing built page assets',
    'Studio 静态资源无效': 'A Studio static asset is invalid',
    '网页操控服务尚未就绪': 'The web control service is not ready',
    '没有收到机器人健康状态': 'No robot health status was received',
    '运行程序与发布版本不一致': 'The running program does not match the release revision',
    '机器人健康检查失败': 'The robot health check failed',
    '等待机器人启动超时': 'Timed out waiting for the robot to start',
    '上次软件更新尚未恢复，请启动机器人服务以完成恢复': 'Start the robot service to complete recovery from the previous update',
    '发布标签与签名清单的源码版本不一致': 'The release tag and signed manifest refer to different source revisions',
    '更新任务编号无效': 'Invalid update job ID',
    'Studio 端口无效': 'Invalid Studio port',
    'Git 操作失败': 'Git operation failed',
    'WonderDuck 仓库地址无效': 'Invalid WonderDuck repository URL',
    '版本包含未识别的子模块': 'The release contains an unrecognized submodule',
    '用户数据路径包含符号链接，已停止更新': 'Update stopped because a user data path contains a symbolic link',
    '用户数据与新版本同名，已停止更新': 'Update stopped because a release file conflicts with user data',
    '请先从官方仓库获取指定版本': 'Fetch the requested version from the official repository first',
    '发布分支无效': 'Invalid release branch',
    'Studio 版本无效': 'Invalid Studio revision',
    'Studio 路径包含符号链接，请先检查': 'Check the symbolic link in the Studio path',
    'Studio 路径不是目录': 'The Studio path is not a directory',
    'Studio 仓库信息无效，请先检查': 'Check the Studio repository metadata',
    '正式版格式为 0.1.0，开发版格式为 0.1.0-dev.1.1': 'Use a version such as 0.1.0 or 0.1.0-dev.1.1',
    '请在设备的 WonderDuck 仓库目录中执行': 'Run this command from the WonderDuck repository on the device',
    '发布版本无效': 'Invalid release revision',
    '发布版本缺少 Studio 子模块': 'The release is missing the Studio submodule',
    '发布版本的 Studio 地址无效': 'Invalid Studio repository URL in the release',
    'Studio 源码状态不完整，请先检查': 'Check the incomplete Studio checkout',
    'Studio 目录在更新期间发生变化，请检查后重试': 'The Studio directory changed during the update. Check it and try again.',
    '源码版本切换未完成，请检查后重试': 'The source checkout did not finish. Check the source tree and try again.',
    '源码已更新；设备模型、动作组和标定数据已保留。': 'Source updated. Device models, action groups, and calibration data were preserved.',
    '找不到设备安装程序': 'The device installer was not found',
    '源码更新失败': 'Source update failed',
    '安装未执行；请检查上述原因后重试。': 'Installation did not run. Check the error and try again.',
    '版本必须为三段数字，例如 0.1.0': 'Use a three-part version such as 0.1.0',
    '发布版本格式应为 0.1.0 或 0.1.0-dev.1.1': 'Use a release version such as 0.1.0 or 0.1.0-dev.1.1',
    '源码仓库存在未提交的修改': 'The source checkout has uncommitted changes',
    'Studio 子模块版本与主仓库记录不一致': 'The Studio revision does not match the main repository',
    '不是 ARM64 Linux 可执行文件或动态库': 'Expected an ARM64 Linux executable or shared library',
    '运行包缺少 Studio 网页入口': 'The runtime bundle is missing a Studio entry page',
    '运行包缺少网页入口': 'The runtime bundle is missing a web entry page',
    'Studio 网页资源缺失': 'Studio web asset missing',
    '发布清单格式不正确': 'Invalid release manifest format',
    '发布标签与版本不一致': 'The release tag and version do not match',
    '不支持的运行包架构或文件名': 'Unsupported runtime bundle architecture or file name',
    '发布清单中的源码版本无效': 'Invalid source revision in the manifest',
    '发布清单中的摘要无效': 'Invalid checksum in the manifest',
    '发布清单中的包大小无效': 'Invalid bundle size in the manifest',
    '发布清单签名校验失败': 'Release manifest signature verification failed',
    '下载进度需要有效的文件大小': 'Download progress requires a valid file size',
    '下载进度': 'Download progress',
    '下载文件超过允许大小': 'The download exceeded the allowed file size',
    '运行包大小与发布清单不一致': 'The runtime bundle size does not match the manifest',
    '运行包 SHA-256 不匹配': 'The runtime bundle SHA-256 checksum does not match',
    '运行包包含无效路径': 'The runtime bundle contains an invalid path',
    '运行包包含越界链接': 'The runtime bundle contains a link outside the installation tree',
    '运行包链接超出资源目录': 'A link in the runtime bundle points outside its resource directory',
    '运行包解压后过大': 'The unpacked runtime bundle is too large',
    '运行包包含不支持的文件类型': 'The runtime bundle contains an unsupported file type',
    '运行包文件位于链接目录下': 'A file in the runtime bundle is under a linked directory',
    '运行包缺少必要文件': 'The runtime bundle is missing a required file',
    '运行包缺少摄像头运行文件': 'The runtime bundle is missing a required camera file',
    '运行文件包含不支持的文件类型': 'A runtime file has an unsupported type',
    '安装状态文件无效': 'Invalid installation state file',
    '安装状态文件格式无效': 'Invalid installation state file format',
    '上次安装未完成，请先重新安装版本 未知': 'Identify the previous version before resuming the interrupted installation',
    '安装路径不能包含符号链接': 'The installation path must not contain symbolic links',
    '安装目录超出运行目录': 'The installation directory is outside the runtime tree',
    '不允许提权创建非运行包目录': 'Privileged creation of a directory outside the runtime bundle is not allowed',
    '安装事务目录异常': 'Invalid installation transaction directory',
    '发现旧版安装事务，请先检查': 'Check the unfinished installation from an older installer',
    '安装事务记录无效': 'Invalid installation transaction record',
    '无法读取安装事务': 'Could not read the installation transaction',
    '安装事务状态无效': 'Invalid installation transaction state',
    '挂载信息格式无效': 'Invalid mount information',
    '运行目录包含挂载点': 'A runtime directory contains a mount point',
    '无法检查运行目录挂载状态': 'Could not check the runtime directory for mount points',
    '安装目标超出运行目录': 'The installation target is outside the runtime tree',
    '不允许清理非运行包目录': 'A directory outside the runtime bundle cannot be removed',
    '安装源或目标超出运行目录': 'The installation source or target is outside the runtime tree',
    '不允许提权替换非运行包文件': 'Privileged replacement of a file outside the runtime bundle is not allowed',
    '安装目标不能是符号链接': 'The installation target must not be a symbolic link',
    '安装目标类型不匹配': 'The installation target has an unexpected file type',
    '旧运行文件超出运行目录': 'An old runtime file is outside the runtime tree',
    '不允许清理非运行包文件': 'A file outside the runtime bundle cannot be removed',
    '已有检测模型不是文件': 'The existing object detection model is not a regular file',
    '摄像头启动脚本不存在或不可执行': 'The camera startup script is missing or not executable',
    '摄像头启动路径无效': 'Invalid camera launcher path',
    '摄像头启动入口不是符号链接，请检查': 'Check the camera launcher because it is not a symbolic link',
    '摄像头运行目录无效': 'Invalid camera runtime directory',
    '旧摄像头构建信息不是普通文件': 'The old camera build information is not a regular file',
    '旧程序目录无效': 'Invalid directory for old executables',
    '旧程序不是普通文件': 'The old executable is not a regular file',
    '无法确认进程状态': 'Could not determine the process state',
    '无法创建机器人锁文件，请先配置设备 sudo 权限': 'Configure device sudo permissions to create the robot lock file',
    '运行程序或其他安装操作正在占用': 'The runtime or another installation is holding the lock',
    '运行程序启动检查失败': 'The runtime startup check failed',
    '无法获取开机配置所需的 sudo 权限': 'Could not obtain the sudo permissions needed for startup configuration',
    '开机配置重载失败': 'Could not reload the startup configuration',
    '开机配置文件内容无效': 'Invalid startup configuration contents',
    '开机配置文件权限无效': 'Invalid startup configuration permissions',
    '开机配置安装失败': 'Could not install the startup configuration',
    '提示音服务安装失败': 'Could not install the prompt sound service',
    '开发版格式为 0.1.0-dev.1.1': 'Use a development version such as 0.1.0-dev.1.1',
    '运行包仅支持 ARM64 设备': 'The runtime bundle requires an ARM64 device',
    '开机配置只支持安装目录': 'Startup configuration requires this installation directory',
    '已清理上次中断的安装暂存文件': 'Removed temporary files from the previous interrupted installation',
    '源码版本与发布包不一致，请先检出发布标签并更新子模块': 'Check out the release tag and update its submodules to match the runtime bundle',
    '当前硬件版本低于运行包要求': 'The hardware version does not meet the runtime bundle requirements',
    '当前镜像版本低于运行包要求': 'The system image does not meet the runtime bundle requirements',
    '源码版本在安装期间发生变化': 'The source revision changed during installation',
    '安装后文件校验失败': 'File verification failed after installation',
    '运行包操作失败': 'Runtime bundle operation failed',
    '等待机器人休息完成超时': 'Timed out waiting for the robot to finish resting',
    '无法确认机器人休息状态': 'Could not confirm the robot rest state',
    '机器人休息状态不一致': 'The robot reported an inconsistent rest state',
    '机器人休息失败': 'The robot failed to finish resting',
    '机器人休息被中断': 'The robot rest sequence was interrupted',
    '无法停止当前控制': 'Could not stop the active controls',
    '机器人拒绝休息': 'The robot rejected the rest request',
    '结果未确认': 'The result could not be confirmed',
    'robotd 维护会话通信失败': 'Communication with the robotd maintenance session failed',
    'robotd 维护会话已断开': 'The robotd maintenance session disconnected',
    'robotd 维护请求失败': 'The robotd maintenance request failed',
    'robotd 维护请求被拒绝': 'robotd rejected the maintenance request',
    'robotd 维护交接未完成': 'The robotd maintenance handover did not finish',
    'robotd 状态查询连接异常': 'The robotd status query connection failed',
    '模型更新控制连接中断': 'The update control connection was interrupted',
    '等待当前休息完成超时': 'Timed out waiting for the current rest sequence',
    '舵机维护或标定正在进行，暂不能更新模型': 'Finish servo maintenance or calibration before updating',
    '无法读取控制权': 'Could not read control ownership',
    '无法登记更新控制会话': 'Could not register the update control session',
    '无法取得模型更新控制权': 'Could not acquire control for the update',
    '模型更新期间控制权已被其他操作接管': 'Another operation took control during the update',
    '模型更新控制会话失效': 'The update control session expired',
    '未知': 'unknown',
}
UPDATE_DIAGNOSTIC_PATTERNS = (
    (r'(WonderDuck|Studio) 源码存在未提交的修改，请先检查，更新未执行',
     r'\1 has uncommitted source changes. Resolve them before updating.'),
    (r'无法获取官方 (main|develop) 分支', r'Could not fetch the official \1 branch'),
    (r'该发布版本不属于官方 (main|develop) 分支(?:，更新未执行)?',
     r'The release does not belong to the official \1 branch'),
    (r'上次安装未完成，请先重新安装版本 (.+)', r'Reinstall version \1 to complete the previous installation'),
    (r'(.+?) 正在运行，请先按现有方式停止程序', r'Stop \1 before installing the update'),
    (r'正在下载运行包（(\d+) 字节）', r'Downloading the runtime bundle (\1 bytes)'),
    (r'运行包 (.+?) 安装完成；请按现有方式启动机器人。',
     r'Runtime bundle \1 installed. Start the robot service when ready.'),
    (r'安装未完成，请重新安装版本 (.+)', r'Installation did not finish. Reinstall version \1'),
    (r'无法创建运行目录 (.+)', r'Could not create runtime directory \1'),
    (r'无法清理旧版运行目录 (.+)', r'Could not remove old runtime directory \1'),
    (r'无法安装运行文件 (.+)', r'Could not install runtime file \1'),
    (r'无法清理旧版运行文件 (.+)', r'Could not remove old runtime file \1'),
    (r'(.+?) 查询失败', r'The \1 query failed'),
    (r'机器人 (.+?) 请求失败', r'The robot \1 request failed'),
    (r'机器人 (.+?) 返回的数据格式无效', r'The robot \1 reply has an invalid format'),
)


def localized_update_line(line, text):
    if text is not TEXT['en']:
        return line
    message, separator, detail = line.partition('：')
    translated = UPDATE_DIAGNOSTICS.get(message)
    if translated is None:
        for pattern, replacement in UPDATE_DIAGNOSTIC_PATTERNS:
            match = re.fullmatch(pattern, message)
            if match:
                translated = match.expand(replacement)
                break
    if translated is None:
        return line
    if separator:
        return translated + ': ' + localized_update_line(detail.lstrip(), text)
    return translated


class DownloadProgressOutput:
    def __init__(self, stream, text):
        self.stream = stream
        self.text = text

    def write(self, value):
        return self.stream.write(value.replace('下载进度：', self.text['download_progress']))

    def flush(self):
        self.stream.flush()

    def isatty(self):
        return self.stream.isatty()


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
        raise ValueError('发布响应无效')
    return data


def check_release(source, version):
    base = source[3] + '/v' + version + '/'
    manifest = json.loads(read_url(base + 'manifest.json', 64 * 1024))
    if (not isinstance(manifest, dict) or manifest.get('tag') != 'v' + version
            or manifest.get('version') != version or manifest.get('architecture') != 'aarch64'
            or manifest.get('archive') != 'aarch64.tar.gz'):
        raise ValueError('发布清单无效')
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
    raise ValueError('找不到完整的软件发布包：' + channel)


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


def prepare_release(runtime, source, version, directory, report, text):
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
    parameters = inspect.signature(runtime.download).parameters
    options = {'progress': True} if 'progress' in parameters else {}
    if 'progress_callback' in parameters:
        options['progress_callback'] = lambda received, size: report('download', progress=received * 100 // size)
    with redirect_stdout(DownloadProgressOutput(sys.stdout, text)):
        runtime.download(base + archive.name, archive, data['size'], data['size'], **options)
    if 'progress_callback' not in parameters:
        report('download', progress=100)
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


def error_text(error, text):
    if isinstance(error, subprocess.CalledProcessError) and error.stderr:
        message = error.stderr.decode('utf-8', errors='replace').strip()
    else:
        message = str(error)
    return '\n'.join(localized_update_line(line, text) for line in message.splitlines())[-1500:]


def install_command(args, text, **kwargs):
    result = command(args, stdout=subprocess.PIPE, **kwargs)
    for line in result.stdout.decode('utf-8', errors='replace').splitlines():
        print(localized_update_line(line, text), flush=True)


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


def record_recovery(stage, version, checkpoint, job_id, *, phase='stopping', language='zh'):
    worktrees, submodule_url, paths = checkpoint
    record = dict(stage=stage.name, version=version, job_id=job_id, phase=phase, language=language,
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


def clear_recovery(text):
    (ROOT / RECOVERY_FILE).unlink(missing_ok=True)
    sync_directory(ROOT)
    helper = ROOT / RECOVERY_HELPER
    try:
        if helper.is_dir():
            for name in ('update.py', 'runtime_release.py'):
                (helper / name).unlink(missing_ok=True)
            helper.rmdir()
    except OSError as error:
        print(error_text(error, text), file=sys.stderr)


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
        text = TEXT['en' if record.get('language') == 'en' else 'zh']
        stage, archive, checkpoint = recovery_checkpoint(runtime, record)
        details = dict(job_id=record['job_id'], failed_phase=record['phase'],
                       error=text['interrupted'])
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
            report_status(record['version'], 'failed', rollback='failed', rollback_error=error_text(error, text), **details)
            raise
        report_status(record['version'], 'failed', rollback='done', **details)
        clear_recovery(text)
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
        print(error_text(error, text), file=sys.stderr)
        return 1


def install_release(runtime, source, version, text, channel, unattended, job_id=None, studio_port=8090,
                    socket_path=ROBOT_SOCKET):
    channel_args = ['--channel', channel] if channel == 'develop' else []
    language = 'en' if text is TEXT['en'] else 'zh'
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
        release = prepare_release(runtime, source, version, stage, report, text)
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
            raise ValueError('发布版本缺少源码更新程序')
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
            record_recovery(stage, version, checkpoint, job_id, language=language)
            print(text['stop'], flush=True)
            service_touched = True
            command(['sudo', '-n', 'systemctl', 'stop', 'robotd'], timeout=35)
        install_command([sys.executable, '-', version, *channel_args], text, input=source_script, timeout=180)
        print(text['install'], flush=True)
        existing_transactions = {directory for directory, _ in runtime.pending(ROOT)}
        record_recovery(stage, version, checkpoint, job_id, phase='applying', language=language)
        runtime_touched = True
        install_command([sys.executable, 'scripts/runtime_release.py', 'install', '--version', version,
                         '--base-url', stage.as_uri(), *channel_args], text, timeout=480)
        phase = 'restart'
        report(phase)
        record_recovery(stage, version, checkpoint, job_id, phase='starting', language=language)
        print(text['start'], flush=True)
        command(['sudo', '-n', 'systemctl', 'start', 'robotd'], timeout=35)
        wait_robot_ready(revision, studio_port, socket_path)
        clear_recovery(text)
        try:
            report('done')
        except OSError as error:
            print(error_text(error, text), file=sys.stderr)
        print(text['done'], flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        cause = error_text(error, text) or text['cancelled']
        result = {'failed_phase': phase, 'error': cause, 'rollback': 'unneeded'}
        if checkpoint and changed_origin:
            try:
                report('rollback', **result)
                if service_touched:
                    with installation_lease(runtime, socket_path):
                        record_recovery(stage, version, checkpoint, job_id,
                                        phase='applying' if runtime_touched else 'stopping', language=language)
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
                    record_recovery(stage, version, checkpoint, job_id, phase='starting', language=language)
                    command(['sudo', '-n', 'systemctl', 'start', 'robotd'], timeout=35)
                    wait_robot_ready(studio_port=studio_port, socket_path=socket_path)
                    clear_recovery(text)
                result['rollback'] = 'done'
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as rollback_error:
                keep_checkpoint = True
                result.update(rollback='failed', rollback_error=error_text(rollback_error, text), checkpoint=str(stage))
        report('failed', **result)
        print(cause, file=sys.stderr)
        if result['rollback'] == 'done':
            print(text['restored'], file=sys.stderr)
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
        text = TEXT[args.lang]
        try:
            record = json.loads((ROOT / RECOVERY_FILE).read_text(encoding='utf-8'))
            if isinstance(record, dict):
                text = TEXT.get(record.get('language'), text)
        except (OSError, ValueError, TypeError):
            pass
        try:
            recover_at_start()
            return 0
        except (OSError, KeyError, TypeError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            print(error_text(error, text), file=sys.stderr)
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
            print(error_text(error, text), file=sys.stderr)
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
