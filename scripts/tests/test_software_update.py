import fcntl
import importlib.util
from contextlib import nullcontext, redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPT = Path(__file__).resolve().parents[1] / 'update.py'
SPEC = importlib.util.spec_from_file_location('software_update_cli', SCRIPT)
UPDATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UPDATE)


class SoftwareUpdateTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.stage = self.root / '.software-update-test'
        self.stage.mkdir()
        (self.stage / 'previous.tar').touch()
        self.binary = self.root / 'robotd'
        self.checkpoint = ([(self.root, '1' * 40, '', 'origin'),
                            (self.root / 'extensions/wonderduck_studio', '2' * 40, '', 'origin')],
                           '', [self.binary])
        self.runtime = SimpleNamespace(OBJECT_MODEL='models/object', ROBOT_LOCK=self.root / 'robot.lock',
                                       ensure_robot_lock=Mock(), check_processes=Mock(),
                                       exclusive_lock=lambda _: nullcontext(),
                                       pending=lambda _: [])
        root_patch = patch.object(UPDATE, 'ROOT', self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)

    def boot(self):
        return UPDATE.recover_at_start()

    def test_boot_restores_interrupted_install_and_clears_journal(self):
        UPDATE.record_recovery(self.stage, '0.1.6', self.checkpoint, 'update', phase='applying')
        with patch.object(UPDATE, 'load_installer', return_value=self.runtime), \
                patch.object(UPDATE, 'checkpoint_paths', return_value=[self.binary]), \
                patch.object(UPDATE, 'restore_checkpoint') as restore:
            self.boot()
        restore.assert_called_once_with(self.runtime, self.stage / 'previous.tar', self.checkpoint, True, boot=True)
        self.assertFalse((self.root / UPDATE.RECOVERY_FILE).exists())
        self.assertFalse(self.stage.exists())
        status = json.loads((self.root / UPDATE.STATUS_FILE).read_text())
        self.assertEqual((status['phase'], status['rollback']), ('failed', 'done'))

    def test_failed_recovery_keeps_the_checkpoint_for_retry(self):
        UPDATE.record_recovery(self.stage, '0.1.6', self.checkpoint, 'update', phase='starting')
        with patch.object(UPDATE, 'load_installer', return_value=self.runtime), \
                patch.object(UPDATE, 'checkpoint_paths', return_value=[self.binary]), \
                patch.object(UPDATE, 'restore_checkpoint', side_effect=OSError('restore failed')):
            with self.assertRaises(OSError):
                self.boot()
        self.assertTrue((self.root / UPDATE.RECOVERY_FILE).exists())
        self.assertTrue((self.stage / 'previous.tar').is_file())
        status = json.loads((self.root / UPDATE.STATUS_FILE).read_text())
        self.assertEqual(status['rollback'], 'failed')

    def test_live_install_only_allows_start_after_runtime_is_ready(self):
        with (self.root / UPDATE.UPDATE_LOCK).open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            UPDATE.record_recovery(self.stage, '0.1.6', self.checkpoint, 'update', phase='applying')
            with self.assertRaises(RuntimeError):
                self.boot()
            UPDATE.record_recovery(self.stage, '0.1.6', self.checkpoint, 'update', phase='starting')
            self.boot()
        self.assertTrue((self.root / UPDATE.RECOVERY_FILE).exists())

    def test_restarts_do_not_proceed_if_rest_was_invalidated(self):
        lease = Mock()
        mock_studio = SimpleNamespace(MaintenanceLease=Mock(return_value=lease))
        mock_rest = SimpleNamespace(ensure_rest=Mock(), rest_status=Mock(return_value=('idle', True)))
        with patch.dict(sys.modules, {'app.robot.studio': mock_studio, 'app.robot.workspace_rest': mock_rest}), \
                patch.object(UPDATE, 'command', return_value=SimpleNamespace(stdout=b'active')):
            with self.assertRaises(RuntimeError):
                with UPDATE.installation_lease(self.runtime, '/unused'):
                    self.fail('robotd must stay running if rest is not confirmed')
        lease.request.assert_called_once_with('acquire')
        lease.close.assert_called_once()

    def test_installation_holds_the_robot_lease_during_stop(self):
        lease = Mock()
        mock_studio = SimpleNamespace(MaintenanceLease=Mock(return_value=lease))
        mock_rest = SimpleNamespace(ensure_rest=Mock(), rest_status=Mock(return_value=('rested', False)))
        with patch.dict(sys.modules, {'app.robot.studio': mock_studio, 'app.robot.workspace_rest': mock_rest}), \
                patch.object(UPDATE, 'command', return_value=SimpleNamespace(stdout=b'active')):
            with UPDATE.installation_lease(self.runtime, '/unused'):
                lease.request.assert_called_once_with('acquire')
                lease.close.assert_not_called()
        lease.close.assert_called_once()

    def test_failed_candidate_with_stale_socket_uses_offline_lock(self):
        stale_socket = self.root / 'robotd.sock'
        stale_socket.touch()
        with patch.object(UPDATE, 'command', return_value=SimpleNamespace(stdout=b'failed')):
            with UPDATE.installation_lease(self.runtime, stale_socket):
                self.runtime.ensure_robot_lock.assert_called_once()
                self.runtime.check_processes.assert_called_once()

    def test_studio_health_rejects_a_missing_page_bundle(self):
        with patch.object(UPDATE, 'read_url', side_effect=[b'{"studio": true}', b'<html></html>']):
            with self.assertRaisesRegex(ValueError, 'Studio 页面'):
                UPDATE.check_studio(8090)

    def test_legacy_installer_diagnostics_follow_the_update_language(self):
        original = '运行包操作失败：发布清单签名校验失败'
        error = subprocess.CalledProcessError(1, ['runtime_release.py'], stderr=original.encode())
        for language, message in (
            ('zh', original),
            ('en', 'Runtime bundle operation failed: Release manifest signature verification failed'),
        ):
            with self.subTest(language=language):
                with redirect_stderr(io.StringIO()) as terminal, \
                        patch.object(UPDATE, 'check_worktrees'), patch.object(UPDATE, 'command'), \
                        patch.object(UPDATE, 'prepare_release', side_effect=error):
                    result = UPDATE.install_release(self.runtime, UPDATE.SOURCES['1'], '0.1.6',
                                                    UPDATE.TEXT[language], 'main', unattended=True)
                self.assertEqual(result, 1)
                self.assertIn(message, terminal.getvalue())
                job = json.loads((self.root / UPDATE.STATUS_FILE).read_text())
                self.assertEqual(job['error'], message)

    def test_legacy_installer_progress_and_completion_in_english(self):
        text = UPDATE.TEXT['en']
        with redirect_stdout(io.StringIO()) as terminal:
            progress = UPDATE.DownloadProgressOutput(sys.stdout, text)
            progress.write('下载进度： 50% (1.0/2.0 MiB)\n')
            child = '运行包 0.1.6 安装完成；请按现有方式启动机器人。\n'.encode()
            with patch.object(UPDATE, 'command', return_value=SimpleNamespace(stdout=child)):
                UPDATE.install_command(['runtime_release.py'], text)
        self.assertEqual(terminal.getvalue().splitlines(), [
            'Download progress:  50% (1.0/2.0 MiB)',
            'Runtime bundle 0.1.6 installed. Start the robot service when ready.',
        ])

    def test_boot_recovery_keeps_the_selected_language(self):
        UPDATE.record_recovery(self.stage, '0.1.6', self.checkpoint, 'update',
                               phase='applying', language='en')
        with patch.object(UPDATE, 'load_installer', return_value=self.runtime), \
                patch.object(UPDATE, 'checkpoint_paths', return_value=[self.binary]), \
                patch.object(UPDATE, 'restore_checkpoint',
                             side_effect=ValueError('无法读取更新前的源码版本：/device')):
            with self.assertRaises(ValueError):
                self.boot()
        job = json.loads((self.root / UPDATE.STATUS_FILE).read_text())
        self.assertEqual(job['error'], 'The software update was interrupted before completion.')
        self.assertEqual(job['rollback_error'], 'Could not read the source revision before the update: /device')

    def test_prepare_release_supports_the_installer_in_the_original_image(self):
        def download(url, destination, limit, expected_size=None):
            destination.write_bytes(b'archive')

        device = self.root / 'device-version.json'
        device.write_text('{"hardware_version": "1.0.0", "image_version": "1.0.0"}')
        manifest = {
            'min_hardware_version': '1.0.0', 'min_image_version': '1.0.0',
            'archive': 'aarch64.tar.gz', 'size': 7, 'sha256': 'checksum',
        }
        runtime = SimpleNamespace(PUBLIC_KEY='public.key', DEVICE_VERSION=device,
                                  verify_signature=Mock(), validate_manifest=Mock(),
                                  version=lambda value: tuple(map(int, value.split('.'))),
                                  download=download, digest_file=Mock(return_value='checksum'))
        report = Mock()
        with patch.object(UPDATE, 'read_url', side_effect=[json.dumps(manifest).encode(), b'signature']):
            actual = UPDATE.prepare_release(runtime, UPDATE.SOURCES['1'], '0.1.6', self.stage,
                                            report, UPDATE.TEXT['en'])
        self.assertEqual(actual, manifest)
        self.assertEqual(report.call_args_list[1].kwargs, {'progress': 100})

    def test_web_check_only_returns_the_digest_of_a_verified_manifest(self):
        (self.root / '.git').mkdir()
        digest = 'a' * 64
        self.runtime.PUBLIC_KEY = 'release.pub'
        self.runtime.verify_signature = Mock()
        self.runtime.validate_manifest = Mock()
        with patch.object(sys, 'argv', ['update.py', '--source', 'gitee', '--check']), \
                patch.object(UPDATE, 'latest_stable_for_browser', return_value='0.1.5'), \
                patch.object(UPDATE, 'load_installer', return_value=self.runtime), \
                patch.object(UPDATE, 'read_url', side_effect=[json.dumps({'sha256': digest}).encode(), b'signature']), \
                redirect_stdout(io.StringIO()) as output:
            self.assertEqual(UPDATE.main(), 0)
        self.runtime.verify_signature.assert_called_once()
        self.runtime.validate_manifest.assert_called_once()
        self.assertEqual(json.loads(output.getvalue()), {'version': '0.1.5', 'sha256': digest})

    def test_changed_confirmed_build_is_rejected_before_downloading(self):
        report = Mock()
        with patch.object(UPDATE, 'fetch_signed_manifest', return_value={'sha256': 'b' * 64}):
            with self.assertRaisesRegex(ValueError, '发布构建已变化'):
                UPDATE.prepare_release(self.runtime, UPDATE.SOURCES['2'], '0.1.5', self.stage,
                                       report, UPDATE.TEXT['zh'], expected_sha256='a' * 64)
        report.assert_not_called()


if __name__ == '__main__':
    unittest.main()
