import fcntl
import importlib.util
from contextlib import nullcontext
import json
from pathlib import Path
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


if __name__ == '__main__':
    unittest.main()
