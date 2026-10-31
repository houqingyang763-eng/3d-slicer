"""Offline guardian lifecycle checks; no Docker, ROS or Slicer is started."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class GuardianTests(unittest.TestCase):
    def setUp(self):
        script = Path(__file__).with_name("ursim_wsl_session.sh").read_text(encoding="utf-8")
        source = script.split("<<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
        self.fcntl = SimpleNamespace(LOCK_EX=2, LOCK_NB=4, LOCK_UN=8, flock=Mock())
        self.namespace = {"__name__": "ursim_guardian_offline_test"}
        with patch.dict(sys.modules, {"fcntl": self.fcntl}):
            exec(compile(source, "ursim_wsl_session.sh:<guardian>", "exec"), self.namespace)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.script_dir = (self.root / "bootstrap").resolve()
        self.script_dir.mkdir()
        self.cache = self.root / "cache"
        self.owner_path = self.cache / "guardian/owner.json"
        self.process = {"pid": 8100, "state": "S", "start_ticks": "77",
                        "command": b"python3\0-\0" + str(self.script_dir).encode()}
        self.common = SimpleNamespace(process=Mock(return_value=self.process))
        self.lifecycle = SimpleNamespace(PROJECT_KEY="project-key", common=lambda: self.common,
                                         cache_root=lambda: self.cache, Docker=Mock(),
                                         verified_container=Mock(return_value={"State": {"Running": False}}),
                                         write_json=self.write_json)
        self.owner = {"pid": 8100, "start_ticks": "77", "project_key": "project-key",
                      "script_dir": str(self.script_dir), "started_at": 1.0, "stopped_at": None}

    def write_json(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def run_main(self, arguments, manager):
        with patch.dict(sys.modules, {"ursim_lifecycle": self.lifecycle}), \
                patch.object(self.namespace["os"], "geteuid", return_value=1000, create=True), \
                patch.object(self.namespace["subprocess"], "run", manager), \
                redirect_stdout(io.StringIO()) as output:
            self.namespace["main"]([str(self.script_dir), *arguments])
        return output.getvalue()

    def test_duplicate_start_reuses_verified_guardian_without_starting_manager(self):
        self.write_json(self.owner_path, self.owner)
        before = self.owner_path.read_bytes()
        self.fcntl.flock.side_effect = BlockingIOError("already held")
        manager = Mock()
        response = self.run_main([], manager)
        manager.assert_not_called()
        self.lifecycle.Docker.assert_not_called()
        self.assertTrue(json.loads(response)["guardian_reused"])
        self.assertEqual(self.owner_path.read_bytes(), before)

    def test_first_start_locks_and_records_identity_before_manager(self):
        manager = Mock()

        def started(command, check):
            self.assertEqual(self.fcntl.flock.call_count, 1)
            recorded = json.loads(self.owner_path.read_text(encoding="utf-8"))
            self.assertEqual(recorded["start_ticks"], "77")
            self.assertIsNone(recorded["stopped_at"])
            self.assertEqual(command[-2:], ["start", "--backend-only"])
            self.assertTrue(check)
            return SimpleNamespace(returncode=0)
        manager.side_effect = started
        self.run_main(["--backend-only"], manager)
        manager.assert_called_once()
        self.assertIsNotNone(json.loads(self.owner_path.read_text(encoding="utf-8"))["stopped_at"])

    def test_manager_failure_marks_guardian_stopped_and_never_polls_docker(self):
        manager = Mock(side_effect=subprocess.CalledProcessError(1, ["test-only-manager"]))
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_main([], manager)
        self.lifecycle.Docker.assert_not_called()
        self.assertIsNotNone(json.loads(self.owner_path.read_text(encoding="utf-8"))["stopped_at"])

    def test_wait_stopped_observes_owned_process_exit_after_lock_release(self):
        self.write_json(self.owner_path, self.owner)
        self.common.process.side_effect = [self.process, None]
        clock = [0.0]
        lock = SimpleNamespace(fileno=lambda: 9)
        with patch.object(self.namespace["time"], "monotonic", side_effect=lambda: clock[0]), \
                patch.object(self.namespace["time"], "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)), \
                patch.object(self.namespace["os"], "kill") as send_signal, redirect_stdout(io.StringIO()) as output:
            self.namespace["wait_stopped"](self.lifecycle, lock, self.owner_path, self.script_dir)
        self.assertTrue(json.loads(output.getvalue())["guardian_stopped"])
        self.assertAlmostEqual(clock[0], 0.1)
        send_signal.assert_not_called()
        self.assertEqual(self.fcntl.flock.call_count, 3)  # acquire, unlock while still alive, acquire

    def test_busy_lock_with_reused_pid_is_refused_without_signals(self):
        self.write_json(self.owner_path, self.owner)
        before = self.owner_path.read_bytes()
        self.fcntl.flock.side_effect = BlockingIOError("already held")
        self.common.process.return_value = {**self.process, "start_ticks": "other-process"}
        with patch.object(self.namespace["os"], "kill") as send_signal, self.assertRaisesRegex(RuntimeError, "mismatch"):
            self.namespace["wait_stopped"](self.lifecycle, SimpleNamespace(fileno=lambda: 9), self.owner_path, self.script_dir)
        send_signal.assert_not_called()
        self.assertEqual(self.owner_path.read_bytes(), before)

    def test_busy_owned_guardian_times_out_without_killing_or_discarding_owner(self):
        self.write_json(self.owner_path, self.owner)
        before = self.owner_path.read_bytes()
        self.fcntl.flock.side_effect = BlockingIOError("still active")
        clock = [0.0]
        with patch.object(self.namespace["time"], "monotonic", side_effect=lambda: clock[0]), \
                patch.object(self.namespace["time"], "sleep", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)), \
                patch.object(self.namespace["os"], "kill") as send_signal, self.assertRaisesRegex(RuntimeError, "did not exit"):
            self.namespace["wait_stopped"](self.lifecycle, SimpleNamespace(fileno=lambda: 9), self.owner_path, self.script_dir, timeout=0.3)
        send_signal.assert_not_called()
        self.assertEqual(self.owner_path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
