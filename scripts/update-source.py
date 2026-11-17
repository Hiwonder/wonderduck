#!/usr/bin/env python3
"""Switch the device source to a release without overwriting device data."""

import importlib.util
import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


STUDIO = 'extensions/wonderduck_studio'
MAIN_URL = 'https://github.com/Hiwonder/wonderduck.git'
STUDIO_URL = 'https://github.com/Hiwonder/wonderduck_studio.git'
GITEE_MAIN_URL = 'https://gitee.com/hiwonder_official/wonderduck.git'
GITEE_STUDIO_URL = 'https://gitee.com/hiwonder_official/wonderduck_studio.git'
MAIN_DATA = (
    'model-current', 'model-defaults', 'model-history', 'models', 'policies',
    'calibrations', 'records', 'actions', 'action-groups', 'exports',
    'pet-detect/models', 'deploy/calibration', 'deploy/imu-pose-standing.json',
)
STUDIO_DATA = ('console/actions', 'console/action-groups', 'console/calibrations',
               'console/servo-layout.json')
VERSION = re.compile(r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\Z')
DEVELOPMENT_VERSION = re.compile(
    r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)-dev\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\Z')
REVISION = re.compile(r'[0-9a-f]{40}\Z')


def git(repo, *arguments, required=True):
    env = os.environ.copy()
    env['GIT_TERMINAL_PROMPT'] = '0'
    result = subprocess.run(['git', '-C', str(repo), *arguments], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if required and result.returncode:
        detail = result.stderr.decode('utf-8', errors='replace').strip()
        raise ValueError(detail or 'Git 操作失败')
    return result


def output(repo, *arguments):
    return git(repo, *arguments).stdout.decode('utf-8').strip()


def official_source(remote):
    for name, main, studio in (('github', MAIN_URL, STUDIO_URL),
                               ('gitee', GITEE_MAIN_URL, GITEE_STUDIO_URL)):
        addresses = {main, main[:-4]} if main.endswith('.git') else {main}
        if remote in addresses:
            return name, main, studio
    raise ValueError('WonderDuck 仓库地址无效')


def protected(name, studio):
    parts = STUDIO_DATA if studio else MAIN_DATA
    return (any(name == part or name.startswith(part + '/') for part in parts)
            or name.lower().endswith(('.onnx', '.rknn')))


def tree(repo, revision, studio=False):
    entries = {}
    for row in git(repo, 'ls-tree', '-r', '-z', '--full-tree', revision).stdout.split(b'\0'):
        if not row:
            continue
        metadata, raw_name = row.split(b'\t', 1)
        mode, kind, object_id = metadata.decode('ascii').split()
        name = raw_name.decode('utf-8')
        if kind == 'commit' and (studio or name != STUDIO or mode != '160000'):
            raise ValueError('版本包含未识别的子模块：' + name)
        entries[name] = (mode, kind, object_id)
    return entries


def protect_existing_data(repo, current, target, studio=False):
    for name in sorted(current.keys() | target.keys()):
        if not protected(name, studio) or current.get(name) == target.get(name):
            continue
        path = repo / name
        # A symlink in the parent chain must not redirect a new tracked file
        # into a different device directory.
        for parent in path.parents:
            if parent == repo:
                break
            if parent.is_symlink():
                raise ValueError('用户数据路径包含符号链接，已停止更新：' + name)
        if path.exists() or path.is_symlink():
            raise ValueError('用户数据与新版本同名，已停止更新：' + name)


def clean(repo, description, ignore_submodules=False):
    options = ('--ignore-submodules=all',) if ignore_submodules else ()
    result = git(repo, 'status', '--porcelain=v1', '--untracked-files=no', *options)
    staged = git(repo, 'diff', '--cached', '--name-only', '-z').stdout
    if result.stdout or staged:
        raise ValueError(description + '存在未提交的修改，请先检查，更新未执行')


def check_fetch_head(content, tag, url):
    lines = content.splitlines()
    fields = lines[0].split('\t', 2) if lines else []
    # Git may omit the .git suffix when describing a fetched tag.
    addresses = {url, url[:-4]} if url.endswith('.git') else {url}
    descriptions = {"tag '" + tag + "' of " + address for address in addresses}
    if len(fields) != 3 or not REVISION.fullmatch(fields[0]) or fields[1] or fields[2] not in descriptions:
        raise ValueError('请先从官方仓库获取指定版本')
    # The branch verification below appends its own fetch to FETCH_HEAD. Allow
    # those entries on retry, while keeping the requested tag as the first entry.
    branch_descriptions = {"branch '" + branch + "' of " + address
                           for branch in ('main', 'develop') for address in addresses}
    for line in lines[1:]:
        entry = line.split('\t', 2)
        if len(entry) != 3 or not REVISION.fullmatch(entry[0]) or entry[1] or entry[2] not in branch_descriptions:
            raise ValueError('请先从官方仓库获取指定版本')


def require_release_branch(repo, revision, branch):
    if branch not in ('main', 'develop'):
        raise ValueError('发布分支无效')
    # Preserve the already-validated release tag in FETCH_HEAD on older Git versions.
    git(repo, 'fetch', '--no-tags', '--no-recurse-submodules', '--append', 'origin',
        'refs/heads/{0}:refs/remotes/origin/{0}'.format(branch))
    if git(repo, 'merge-base', '--is-ancestor', revision,
           'refs/remotes/origin/' + branch, required=False).returncode:
        raise ValueError('该发布版本不属于官方 {} 分支，更新未执行'.format(branch))


def require_main_release(repo, revision):
    require_release_branch(repo, revision, 'main')


def ensure_revision(repo, revision, url):
    if not REVISION.fullmatch(revision):
        raise ValueError('Studio 版本无效')
    if git(repo, 'cat-file', '-e', revision + '^{commit}', required=False).returncode:
        git(repo, 'fetch', '--no-tags', '--no-recurse-submodules', '--', url, revision)
    if output(repo, 'rev-parse', '--verify', revision + '^{commit}') != revision:
        raise ValueError('Studio 版本无效')


def studio_repository(root):
    path = root / STUDIO
    for part in (root / 'extensions', path):
        if part.is_symlink():
            raise ValueError('Studio 路径包含符号链接，请先检查：' + str(part))
    if not path.exists():
        return path, False
    if not path.is_dir():
        raise ValueError('Studio 路径不是目录：' + str(path))
    result = git(path, 'rev-parse', '--show-toplevel', required=False)
    if result.returncode == 0:
        top = Path(result.stdout.decode('utf-8').strip()).resolve()
        if top == path.resolve():
            return path, True
    metadata = path / '.git'
    if metadata.exists() or metadata.is_symlink():
        raise ValueError('Studio 仓库信息无效，请先检查：' + str(path))
    return path, False


def update(root, version, channel='main'):
    pattern = VERSION if channel == 'main' else DEVELOPMENT_VERSION if channel == 'develop' else None
    if pattern is None or not pattern.fullmatch(version):
        raise ValueError('正式版格式为 0.1.0，开发版格式为 0.1.0-dev.1.1')
    root = Path(root).resolve()
    if Path(output(root, 'rev-parse', '--show-toplevel')).resolve() != root:
        raise ValueError('请在设备的 WonderDuck 仓库目录中执行')

    _, main_url, studio_url = official_source(output(root, 'remote', 'get-url', 'origin'))
    tag = 'v' + version
    fetch_head = Path(output(root, 'rev-parse', '--git-path', 'FETCH_HEAD'))
    if not fetch_head.is_absolute():
        fetch_head = root / fetch_head
    check_fetch_head(fetch_head.read_text(encoding='utf-8'), tag, main_url)
    revision = output(root, 'rev-parse', '--verify', 'FETCH_HEAD^{commit}')
    if not REVISION.fullmatch(revision):
        raise ValueError('发布版本无效')
    require_release_branch(root, revision, channel)
    # A previous interrupted update can leave the Studio checkout at its old
    # commit; inspect that worktree separately so retry remains possible.
    clean(root, 'WonderDuck 源码', ignore_submodules=True)
    current = tree(root, 'HEAD')
    target = tree(root, revision)
    protect_existing_data(root, current, target)
    studio = target.get(STUDIO)
    if studio is None or studio[:2] != ('160000', 'commit'):
        raise ValueError('发布版本缺少 Studio 子模块')
    studio_revision = studio[2]
    declared_path = output(root, 'config', '--blob', revision + ':.gitmodules', '--get',
                           'submodule.' + STUDIO + '.path')
    declared_url = output(root, 'config', '--blob', revision + ':.gitmodules', '--get',
                          'submodule.' + STUDIO + '.url')
    if declared_path != STUDIO or declared_url not in ('../wonderduck_studio.git', STUDIO_URL, GITEE_STUDIO_URL):
        raise ValueError('发布版本的 Studio 地址无效')

    studio_path, initialized = studio_repository(root)
    if initialized:
        clean(studio_path, 'Studio 源码')
        if git(studio_path, 'rev-parse', '--verify', 'HEAD^{commit}', required=False).returncode == 0:
            studio_current = tree(studio_path, 'HEAD', studio=True)
        elif git(studio_path, 'ls-files', '-z').stdout:
            raise ValueError('Studio 源码状态不完整，请先检查')
        else:
            studio_current = {}
        ensure_revision(studio_path, studio_revision, studio_url)
        studio_target = tree(studio_path, studio_revision, studio=True)
        protect_existing_data(studio_path, studio_current, studio_target, studio=True)
    else:
        # Inspect an uninitialized Studio commit before touching either worktree.
        with tempfile.TemporaryDirectory(prefix='wonderduck-studio-check-') as directory:
            temporary = Path(directory)
            git(temporary, 'init', '--bare', '--quiet')
            ensure_revision(temporary, studio_revision, studio_url)
            studio_target = tree(temporary, studio_revision, studio=True)
            protect_existing_data(studio_path, {}, studio_target, studio=True)

    # These Git commands preserve ignored as well as untracked same-name files.
    # If either checkout fails, report it and leave the user files in place.
    git(root, 'checkout', '--detach', '--no-overwrite-ignore', revision)
    git(root, 'submodule', 'init', '--', STUDIO)
    git(root, 'config', '--local', 'submodule.' + STUDIO + '.url', studio_url)
    if not initialized:
        # Recheck after the main checkout in case the directory changed.
        _, appeared = studio_repository(root)
        if appeared:
            raise ValueError('Studio 目录在更新期间发生变化，请检查后重试')
        # init and fetch do not populate the worktree. The checkout below is
        # protected even if a user creates a file during initialization.
        studio_path.mkdir(parents=True, exist_ok=True)
        git(studio_path, 'init', '--quiet')
        git(studio_path, 'remote', 'add', 'origin', studio_url)
        ensure_revision(studio_path, studio_revision, studio_url)
    else:
        remote = git(studio_path, 'remote', 'get-url', 'origin', required=False)
        git(studio_path, 'remote', 'set-url' if remote.returncode == 0 else 'add', 'origin', studio_url)
    git(studio_path, 'checkout', '--detach', '--no-overwrite-ignore', studio_revision)
    if output(root, 'rev-parse', 'HEAD') != revision or output(studio_path, 'rev-parse', 'HEAD') != studio_revision:
        raise ValueError('源码版本切换未完成，请检查后重试')
    print('源码已更新；设备模型、动作组和标定数据已保留。', flush=True)


def main():
    try:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument('version')
        parser.add_argument('--channel', choices=('main', 'develop'), default='main')
        args = parser.parse_args()
        root = Path.cwd().resolve()
        source = root / 'scripts/runtime_release.py'
        spec = importlib.util.spec_from_file_location('device_runtime_release', source)
        if spec is None or spec.loader is None:
            raise ValueError('找不到设备安装程序')
        runtime = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runtime)
        runtime.ensure_robot_lock()
        with runtime.exclusive_lock(root / '.runtime-install.lock', create=True):
            with runtime.exclusive_lock(runtime.ROBOT_LOCK):
                runtime.check_processes()
                update(root, args.version, args.channel)
        return 0
    except (OSError, ValueError, UnicodeError, ImportError, AttributeError, subprocess.SubprocessError) as error:
        print('源码更新失败：' + str(error), file=sys.stderr)
        print('安装未执行；请检查上述原因后重试。', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
