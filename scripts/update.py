#!/usr/bin/env python3
"""Interactively install an official WonderDuck release."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import urllib.error
import urllib.request


ROOT = Path('/home/wonderduck/wonderduck')
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


def read_url(url, limit):
    request = urllib.request.Request(url, headers={'User-Agent': 'WonderDuck-Updater', 'Accept': 'application/json'})
    with urllib.request.urlopen(request, timeout=15) as response:
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


def command(args, **kwargs):
    env = dict(os.environ, GIT_TERMINAL_PROMPT='0')
    return subprocess.run(args, cwd=ROOT, env=env, check=True, **kwargs)


def install(source, version, text, channel):
    channel_args = ['--channel', channel] if channel == 'develop' else []
    stopped = False
    try:
        command(['sudo', '-v'])
        print(text['fetch'], flush=True)
        command(['git', 'remote', 'set-url', 'origin', source[1]])
        command(['git', 'fetch', '--no-tags', '--no-recurse-submodules', 'origin',
                 '+refs/heads/{0}:refs/remotes/origin/{0}'.format(channel)])
        command(['git', 'fetch', '--no-tags', '--no-recurse-submodules', 'origin', 'refs/tags/v' + version])
        command(['git', 'merge-base', '--is-ancestor', 'FETCH_HEAD', 'refs/remotes/origin/' + channel])
        source_script = command(['git', 'show', 'FETCH_HEAD:scripts/update-source.py'],
                                stdout=subprocess.PIPE).stdout
        if not source_script:
            raise ValueError('empty source updater')
        for path, label in ((ROOT, 'WonderDuck'), (ROOT / 'extensions/wonderduck_studio', 'WonderDuck Studio')):
            if path != ROOT and not (path / '.git').exists():
                continue
            status = command(['git', '-C', str(path), 'status', '--porcelain=v1',
                              '--untracked-files=no', '--ignore-submodules=all'], stdout=subprocess.PIPE).stdout
            if status.strip():
                raise ValueError(text['dirty'].format(label))
        reload_needed = command(['systemctl', 'show', 'robotd', '-p', 'NeedDaemonReload', '--value'],
                                stdout=subprocess.PIPE).stdout.strip()
        if reload_needed == b'yes':
            print(text['reload'], flush=True)
            command(['sudo', 'systemctl', 'daemon-reload'])
        print(text['stop'], flush=True)
        command(['sudo', 'systemctl', 'stop', 'robotd'])
        stopped = True
        command([sys.executable, '-', version, *channel_args], input=source_script)
        print(text['install'], flush=True)
        command([sys.executable, 'scripts/runtime_release.py', 'install', '--version', version, *channel_args])
        print(text['start'], flush=True)
        command(['sudo', 'systemctl', 'start', 'robotd'])
        stopped = False
        print(text['done'], flush=True)
        return 0
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        print(text['failed'], file=sys.stderr)
        if stopped:
            print(text['stopped'], file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n' + text['cancelled'], file=sys.stderr)
        if stopped:
            print(text['stopped'], file=sys.stderr)
        return 130


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lang', choices=('zh', 'en'), default='zh')
    parser.add_argument('--channel', choices=('main', 'develop'), default='main', help=argparse.SUPPRESS)
    args = parser.parse_args()
    text = TEXT[args.lang]
    prefix = 'dev_' if args.channel == 'develop' else ''
    if not (ROOT / '.git').exists():
        print(text['no_repo'], file=sys.stderr)
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
