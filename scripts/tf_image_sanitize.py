"""Remove device-specific network and SSH state from an offline image copy."""

import configparser
import io
import json
import posixpath
import re
import shlex


class ImageFiles:
    def __init__(self, image, debugfs, write_file, tempdir):
        self.image = image
        self.debugfs = debugfs
        self.write_file = write_file
        self.tempdir = tempdir
        self.sequence = 0

    @staticmethod
    def quoted(path):
        if any(c in path for c in ('"', '\\', '\n', '\r', '\x00')):
            raise RuntimeError('镜像包含无法安全处理的文件名')
        return '"' + path + '"'

    def call(self, command, write=False):
        try:
            output, error = self.debugfs(self.image, command, write=write)
        except (RuntimeError, UnicodeError) as exc:
            raise RuntimeError('无法安全检查或修改镜像中的文件') from exc
        return output, error

    def stat(self, path):
        output, error = self.call('stat ' + self.quoted(path))
        if 'File not found' in error:
            return None
        match = re.match(r'Inode:\s*(\d+)\s+Type:\s*(\S+)', output)
        links = re.search(r'(?m)^Links:\s*(\d+)', output)
        if error or not match or not links:
            raise RuntimeError('无法安全检查镜像中的文件')
        return int(match.group(1)), match.group(2), int(links.group(1))

    def check_parents(self, path):
        parts = path.strip('/').split('/')[:-1]
        current = ''
        for part in parts:
            current += '/' + part
            item = self.stat(current)
            if item is None:
                return False
            if item[1] != 'directory':
                raise RuntimeError('镜像敏感文件的父目录不是普通目录')
        return True

    def info(self, path):
        if not self.check_parents(path):
            return None
        return self.stat(path)

    def resolve_directory(self, path):
        """Resolve home-directory links without ever leaving the image."""
        for _ in range(16):
            components = [part for part in path.split('/') if part]
            current = ''
            for index, part in enumerate(components):
                current += '/' + part
                item = self.stat(current)
                if item is None:
                    return None
                if item[1] == 'symlink':
                    output, error = self.call('stat ' + self.quoted(current))
                    match = re.search(r'Fast link dest: "([^"\n]+)"', output)
                    if error or not match:
                        raise RuntimeError('用户目录包含无法安全处理的链接')
                    target = match.group(1)
                    base = target if target.startswith('/') else posixpath.join(posixpath.dirname(current), target)
                    path = posixpath.normpath(posixpath.join(base, *components[index + 1:]))
                    break
                if item[1] != 'directory':
                    return None
            else:
                return current or '/'
        raise RuntimeError('用户目录链接过多，无法安全脱敏')

    def entries(self, path):
        item = self.info(path)
        if item is None:
            return []
        if item[1] != 'directory':
            raise RuntimeError('镜像敏感目录不是普通目录')
        output, error = self.call('ls -p ' + self.quoted(path))
        if error:
            raise RuntimeError('无法安全列出镜像中的目录')
        entries = []
        for line in output.splitlines():
            if not line:
                continue
            fields = line.split('/')
            if len(fields) != 8 or fields[0] or fields[-1] or not fields[1].isdigit():
                raise RuntimeError('镜像敏感目录包含无法安全处理的文件名')
            name = fields[5]
            if name in ('.', '..'):
                continue
            if not name or name in ('.', '..'):
                raise RuntimeError('镜像敏感目录包含无法安全处理的文件名')
            self.quoted(name)
            entries.append(name)
        return entries

    def read(self, path):
        item = self.info(path)
        if item is None:
            return None
        if item[1] != 'regular':
            raise RuntimeError('镜像中的配置文件不是普通文件')
        data, error = self.call('cat ' + self.quoted(path))
        if error:
            raise RuntimeError('无法读取镜像中的配置文件')
        return data

    def remove(self, path):
        item = self.info(path)
        if item is None:
            return False
        inode, kind, links = item
        if kind == 'directory':
            for name in self.entries(path):
                self.remove(path + '/' + name)
            command = 'rmdir '
        elif kind == 'regular':
            if links != 1:
                raise RuntimeError('镜像敏感文件包含硬链接，无法安全脱敏')
            command = 'rm '
        elif kind == 'symlink' and path == '/var/lib/dbus/machine-id':
            # The standard D-Bus link points to /etc/machine-id, which is
            # reset below. Refuse nonstandard targets instead of following
            # or silently leaving their contents elsewhere in the image.
            output, error = self.call('stat ' + self.quoted(path))
            if error or 'Fast link dest: "/etc/machine-id"' not in output:
                raise RuntimeError('D-Bus machine-id 使用了自定义链接，无法安全脱敏')
            command = 'rm '
        else:
            raise RuntimeError('镜像敏感目录包含不支持的文件类型')
        _, error = self.call(command + self.quoted(path), write=True)
        if error or self.info(path) is not None:
            raise RuntimeError('无法清除镜像中的敏感文件')
        # Inline file data can live inside an inode; clear the freed inode too.
        _, error = self.call('clri <{}>'.format(inode), write=True)
        if error:
            raise RuntimeError('无法清除镜像中的已释放 inode')
        return True

    def replace(self, path, data, mode):
        item = self.info(path)
        if item and item[1] != 'regular':
            raise RuntimeError('镜像中的配置文件不是普通文件')
        self.remove(path)
        if not self.check_parents(path):
            raise RuntimeError('镜像中缺少配置目录')
        self.sequence += 1
        staged = self.tempdir / ('sanitize-{}'.format(self.sequence))
        staged.write_text(data, encoding='utf-8')
        try:
            self.write_file(self.image, staged, path)
            _, error = self.call('set_inode_field {} mode {}'.format(self.quoted(path), mode), write=True)
            if error:
                raise RuntimeError('无法设置镜像中的配置文件权限')
        finally:
            staged.unlink(missing_ok=True)


def rewrite_nm_profile(data):
    config = configparser.ConfigParser(interpolation=None, strict=False)
    config.optionxform = str
    try:
        config.read_string(data)
    except configparser.Error as exc:
        raise RuntimeError('无法安全解析网络配置') from exc
    kind = config.get('connection', 'type', fallback='')
    wireless = [name for name in ('wifi', '802-11-wireless') if config.has_section(name)]
    if kind not in ('wifi', '802-11-wireless') and not wireless:
        if not kind:
            raise RuntimeError('网络配置缺少连接类型，无法安全脱敏')
        return data
    if len(wireless) != 1:
        raise RuntimeError('Wi-Fi 配置无法安全识别')
    if config.get(wireless[0], 'mode', fallback='infrastructure').lower() != 'ap':
        return None
    security = [name for name in ('wifi-security', '802-11-wireless-security') if config.has_section(name)]
    if len(security) > 1:
        raise RuntimeError('热点配置无法安全识别')
    section = security[0] if security else 'wifi-security'
    if not security:
        config.add_section(section)
    for key in list(config[section]):
        if key in ('psk', 'psk-flags', 'leap-password', 'leap-password-flags',
                   'sae-password', 'wep-key0', 'wep-key1', 'wep-key2', 'wep-key3', 'wep-key-flags'):
            config.remove_option(section, key)
    config[section]['key-mgmt'] = 'wpa-psk'
    config[section]['psk'] = 'hiwonder'
    if config.has_section('802-1x'):
        config.remove_section('802-1x')
    output = io.StringIO()
    config.write(output, space_around_delimiters=False)
    return output.getvalue()


def rewrite_wpa(data):
    # Network blocks are the only wpa_supplicant entries holding saved networks.
    # Preserve global settings and AP blocks; all station blocks are dropped.
    pattern = re.compile(r'(?m)^[ \t]*network\s*=\s*\{')
    pieces, pos = [], 0
    while True:
        match = pattern.search(data, pos)
        if not match:
            pieces.append(data[pos:])
            break
        pieces.append(data[pos:match.start()])
        quoted = escaped = comment = False
        end = None
        for index in range(match.end(), len(data)):
            char = data[index]
            if comment:
                if char == '\n':
                    comment = False
                continue
            if quoted:
                if escaped:
                    escaped = False
                elif char == '\\':
                    escaped = True
                elif char == '"':
                    quoted = False
                continue
            if char == '#':
                comment = True
            elif char == '"':
                quoted = True
            elif char == '{':
                raise RuntimeError('无法安全解析无线网络配置')
            elif char == '}':
                end = index + 1
                break
        if end is None:
            raise RuntimeError('无法安全解析无线网络配置')
        block = data[match.start():end]
        modes = re.findall(r'(?m)^\s*mode\s*=\s*(\d+)\s*(?:#.*)?$', block)
        if len(modes) > 1:
            raise RuntimeError('无法安全识别无线热点配置')
        if modes == ['2']:
            block = re.sub(r'(?m)^\s*(?:psk|key_mgmt|wep_key[0-3]|password|sae_password)\s*=.*\n?', '', block)
            block = block[:-1] + '\n    key_mgmt=WPA-PSK\n    psk="hiwonder"\n}'
            pieces.append(block)
        pos = end
    return ''.join(pieces)


def clear_recordings(files, root):
    """Remove recognized record_velocity sessions and loose recordings."""
    item = files.info(root)
    if item is None:
        return
    if item[1] != 'directory':
        raise RuntimeError('录制目录不是普通目录，无法安全清理')

    def clean_directory(path):
        names = set(files.entries(path))
        if 'request.json' in names and 'params.toml' in names:
            try:
                request = json.loads(files.read(path + '/request.json'))
            except (TypeError, ValueError) as exc:
                raise RuntimeError('无法识别录制会话，已保留其文件') from exc
            if not isinstance(request, dict):
                raise RuntimeError('无法识别录制会话，已保留其文件')
            policy = request.get('policy_path')
            source = request.get('config_source_path')
            if not all(isinstance(value, str) and value.startswith('/') and value != root
                       and not value.startswith(root + '/')
                       for value in (policy, source)):
                raise RuntimeError('录制会话中包含无法确认来源的模型或配置')
            expected = {'params.toml', 'request.json', 'robotd.log', 'registers-before.jsonl',
                        'registers-after.jsonl', 'run-result.json', 'servo-diagnostics-comparison.json'}
            filename = request.get('recording_filename')
            if isinstance(filename, str) and filename.endswith('.jsonl') and '/' not in filename:
                expected.add(filename)
            if 'velocity.onnx' in names:
                alias = request.get('velocity_alias')
                if not isinstance(alias, dict) or alias.get('path') != path + '/velocity.onnx':
                    raise RuntimeError('录制会话中存在无法确认来源的模型')
                expected.add('velocity.onnx')
            if not names <= expected:
                raise RuntimeError('录制目录包含无法识别的文件，已保留以免误删用户数据')
            files.remove(path)
            return
        for name in names:
            child = path + '/' + name
            info = files.info(child)
            if info[1] == 'directory':
                contents = set(files.entries(child))
                if name in ('deployment-like', 'audio', 'system-test') or {'request.json', 'params.toml'} <= contents:
                    clean_directory(child)
                else:
                    raise RuntimeError('录制目录包含无法识别的文件，已保留以免误删用户数据')
            elif info[1] == 'regular' and name.endswith(('.jsonl', '.jsonl.gz', '.log', '.wav')):
                files.remove(child)
            else:
                raise RuntimeError('录制目录包含无法识别的文件，已保留以免误删用户数据')
        files.remove(path)

    # Refuse to remove snapshots if the robot's active policy configuration
    # actually points into this recording directory.
    for config in ('/etc/robot/robotd.toml', '/etc/default/robotd'):
        data = files.read(config)
        if data and root + '/' in data:
            raise RuntimeError('当前机器人配置引用录制目录，无法安全清理')
    overrides = '/etc/systemd/system/robotd.service.d'
    for name in files.entries(overrides):
        if name.endswith('.conf') and root + '/' in files.read(overrides + '/' + name):
            raise RuntimeError('机器人服务配置引用录制目录，无法安全清理')
    clean_directory(root)


def sanitize(image, tempdir, debugfs, write_file):
    files = ImageFiles(image, debugfs, write_file, tempdir)
    # These directories are historical network/identity archives, not the
    # active configuration. They can retain copies of old credentials.
    for path in ('/root/network-backups', '/var/lib/system-identity'):
        files.remove(path)
    for root in ('/home/wonderduck/records', '/home/wonderduck/wonderduck/records'):
        clear_recordings(files, root)
    adapter = re.compile(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}\Z', re.IGNORECASE)
    for name in files.entries('/var/lib/bluetooth'):
        if adapter.fullmatch(name):
            files.remove('/var/lib/bluetooth/' + name)

    nm_dir = '/etc/NetworkManager/system-connections'
    for name in files.entries(nm_dir):
        path = nm_dir + '/' + name
        result = rewrite_nm_profile(files.read(path))
        if result is None:
            files.remove(path)
        elif result != files.read(path):
            files.replace(path, result, '0100600')

    nm_state = '/var/lib/NetworkManager'
    for name in files.entries(nm_state):
        if (name == 'secret_key' or name.startswith(('seen-bssids', 'timestamps'))
                or re.fullmatch(r'internal-.*wlan.*\.lease.*', name)
                or re.fullmatch(r'dnsmasq-wlan.*\.leases.*', name)):
            files.remove(nm_state + '/' + name)

    wpa_dir = '/etc/wpa_supplicant'
    for name in files.entries(wpa_dir):
        if name.endswith('.conf'):
            path = wpa_dir + '/' + name
            old = files.read(path)
            new = rewrite_wpa(old)
            if old != new:
                files.replace(path, new, '0100600')
        elif '.conf.' in name or name.endswith('.conf~'):
            files.remove(wpa_dir + '/' + name)

    # A custom netplan Wi-Fi tree needs explicit review to avoid accidentally
    # removing an AP definition or leaving station credentials in the image.
    for name in files.entries('/etc/netplan'):
        if name.endswith(('.yaml', '.yml')):
            data = files.read('/etc/netplan/' + name)
            if re.search(r'''(?mi)(?:^|[\s,{])["']?wifis["']?\s*:''', data):
                raise RuntimeError('镜像含 netplan Wi-Fi 配置，请先确认热点与私人网络的处理方式')

    wifi_config = files.info('/etc/wifi-manager')
    wifi_state = files.info('/var/lib/wifi-manager')
    if wifi_config:
        files.replace('/etc/wifi-manager/hotspot.psk', 'hiwonder\n', '0100600')
        if not wifi_state:
            _, error = files.call('mkdir "/var/lib/wifi-manager"', write=True)
            wifi_state = files.info('/var/lib/wifi-manager')
            if error or not wifi_state or wifi_state[1] != 'directory':
                raise RuntimeError('无法创建热点状态目录')
    if wifi_state:
        files.replace('/var/lib/wifi-manager/state.json', '{"mode":"ap"}\n', '0100600')

    ssh_dir = '/etc/ssh'
    for name in files.entries(ssh_dir):
        if name.startswith('ssh_host_'):
            files.remove(ssh_dir + '/' + name)
    for path in ('/etc/ssh/sshd_config',):
        data = files.read(path)
        if data is not None:
            check_sshd_config(data)
    for name in files.entries('/etc/ssh/sshd_config.d'):
        if name.endswith('.conf'):
            check_sshd_config(files.read('/etc/ssh/sshd_config.d/' + name))

    homes = {'/root', '/etc/skel'}
    passwd = files.read('/etc/passwd')
    if passwd:
        for line in passwd.splitlines():
            fields = line.split(':')
            if len(fields) >= 7 and fields[2].isdigit():
                home = fields[5]
                if home == '/':
                    continue
                if home.startswith('/') and all(part not in ('', '.', '..') for part in home[1:].split('/')):
                    homes.add(home)
                elif home:
                    raise RuntimeError('用户目录路径无法安全处理')
    for name in files.entries('/home'):
        homes.add('/home/' + name)
    resolved_homes = set()
    for home in homes:
        resolved = files.resolve_directory(home)
        if resolved and resolved != '/':
            resolved_homes.add(resolved)
    for home in resolved_homes:
        ssh = home + '/.ssh'
        for name in files.entries(ssh):
            if name.startswith(('authorized_keys', 'id_', 'known_hosts')):
                files.remove(ssh + '/' + name)
        for name in ('.git-credentials', '.gitconfig', '.config/git/config', '.config/git/credentials',
                     '.netrc', '.bash_history', '.zsh_history', '.viminfo',
                     '.python_history', '.lesshst', '.mysql_history', '.sqlite_history',
                     '.node_repl_history', '.bluetoothctl_history', '.cache/.bluetoothctl_history',
                     '.cache/bluetoothctl_history', '.cache/bluetoothctl/history',
                     '.local/share/bluetoothctl/history', '.local/share/fish/fish_history'):
            files.remove(home + '/' + name)

    if files.info('/var/lib/systemd/random-seed'):
        files.remove('/var/lib/systemd/random-seed')
    if files.info('/etc/machine-id'):
        files.replace('/etc/machine-id', 'uninitialized\n', '0100644')
    if files.info('/var/lib/dbus/machine-id'):
        files.remove('/var/lib/dbus/machine-id')
    return files


def check_sshd_config(data):
    for line in data.splitlines():
        try:
            words = shlex.split(line, comments=True)
        except ValueError as exc:
            raise RuntimeError('无法安全解析 SSH 服务配置') from exc
        if not words:
            continue
        key, values = words[0].lower(), words[1:]
        if key == 'authorizedkeysfile':
            allowed = {'.ssh/authorized_keys', '.ssh/authorized_keys2',
                       '%h/.ssh/authorized_keys', '%h/.ssh/authorized_keys2', 'none'}
            if not values or any(value not in allowed for value in values):
                raise RuntimeError('SSH 授权文件使用了自定义路径，无法安全脱敏')
        if key == 'authorizedkeyscommand' and values != ['none']:
            raise RuntimeError('SSH 授权使用了自定义命令，无法安全脱敏')
        if key == 'hostkey' and any(not value.startswith('/etc/ssh/ssh_host_') for value in values):
            raise RuntimeError('SSH 主机密钥使用了自定义路径，无法安全脱敏')
        if key == 'include' and any(value != '/etc/ssh/sshd_config.d/*.conf' for value in values):
            raise RuntimeError('SSH 服务配置使用了自定义包含文件，无法安全脱敏')
