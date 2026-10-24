"""Regression tests for lifecycle ownership/readiness; no ROS, GUI or signals.

Run on Linux with:
  python3 -B -m unittest discover -s ur12e_slicer/bootstrap \
    -p test_linux_lifecycle.py -v
All subprocess launch, process inspection and process-group signals used by the
tested orchestration paths are mocked. JSON fixtures live in a temporary tree.
"""
import contextlib
import io
import json
from pathlib import Path
import runpy
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

if sys.platform == "linux":
    import linux_lifecycle as lifecycle
else:
    lifecycle = None


@unittest.skipUnless(lifecycle is not None, "Linux lifecycle uses fcntl and /proc")
class LinuxLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="ur12e lifecycle ")
        self.addCleanup(self.temporary.cleanup)
        self.project = Path(self.temporary.name) / "ur12e_slicer"
        self.runtime = self.project / "runtime"
        self.runtime.mkdir(parents=True)
        changed = patch.multiple(lifecycle, PROJECT=self.project,
                                 REPO=self.project.parent, RUNTIME=self.runtime)
        changed.start()
        self.addCleanup(changed.stop)
        self.pid = 8100
        self.owner = {"pid": self.pid, "group": self.pid,
                      "start_ticks": "1000", "executable": "/operator/old/Slicer"}
        self.proc = {"pid": self.pid, "group": self.pid, "state": "S",
                     "start_ticks": "1000",
                     "command": b"/apps/Slicer\0--python-script\0" +
                     str(self.project / "slicer/start_slicer.py").encode()}

    def save(self, relative, value):
        path = self.runtime / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def bridge_fixture(self, panel=None):
        self.save("linux_slicer.json", self.owner)
        self.save("slicer_bridge/session.json", {"session": "test-session"})
        self.save("slicer_bridge/state_test-session.json", {
            "pid": self.pid, "session": "test-session",
            "task": "ur12e_slicer", "heartbeat": 99.0})
        if panel is not None:
            self.save("slicer_session.json", panel)

    def read_status(self):
        with patch.object(lifecycle, "process", return_value=self.proc), \
             patch.object(lifecycle.time, "time", return_value=100.0):
            return lifecycle.slicer_status()

    def test_bridge_alive_without_panel_is_running_but_not_ready(self):
        self.bridge_fixture()
        result = self.read_status()
        self.assertTrue(result["running"])
        self.assertFalse(result["ready"])

    def test_stale_panel_heartbeat_does_not_pass_fresh_bridge(self):
        self.bridge_fixture({"pid": self.pid, "heartbeat": 80.0})
        result = self.read_status()
        self.assertTrue(result["running"])
        self.assertFalse(result["ready"])

    def test_fresh_panel_for_different_pid_is_not_ready(self):
        self.bridge_fixture({"pid": self.pid + 1, "heartbeat": 99.0})
        self.assertFalse(self.read_status()["ready"])

    def test_fresh_bridge_and_same_pid_panel_are_ready(self):
        self.bridge_fixture({"pid": self.pid, "heartbeat": 99.0})
        self.assertTrue(self.read_status()["ready"])

    def test_new_backend_failure_does_not_close_existing_frontend(self):
        self.bridge_fixture({"pid": self.pid, "heartbeat": 99.0})
        with patch.object(lifecycle, "slicer_executable", return_value=Path("/apps/Slicer")), \
             patch.object(lifecycle, "backend_status", return_value={"running": False, "ready": False}), \
             patch.object(lifecycle, "slicer_status", return_value={"running": True, "ready": True}), \
             patch.object(lifecycle, "wait_ready", side_effect=RuntimeError("backend startup failed")), \
             patch.object(lifecycle, "stop_slicer") as stop_frontend, \
             patch.object(lifecycle.subprocess, "Popen") as launch, \
             patch.object(lifecycle.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as stop_backend:
            with self.assertRaisesRegex(RuntimeError, "backend startup failed"):
                lifecycle.start_all()
        stop_frontend.assert_not_called()
        launch.assert_called_once()
        stop_backend.assert_called_once_with(["bash", str(self.project / "bootstrap/stop_simulation.sh")])
        self.assertTrue((self.runtime / "linux_slicer.json").exists())

    def test_reused_backend_failure_does_not_stop_reused_components(self):
        with patch.object(lifecycle, "slicer_executable", return_value=Path("/apps/Slicer")), \
             patch.object(lifecycle, "backend_status", return_value={"running": True, "ready": False}), \
             patch.object(lifecycle, "wait_ready", side_effect=RuntimeError("backend not ready")), \
             patch.object(lifecycle, "stop_slicer") as stop_frontend, \
             patch.object(lifecycle.subprocess, "Popen") as launch, \
             patch.object(lifecycle.subprocess, "run") as stop_backend:
            with self.assertRaisesRegex(RuntimeError, "backend not ready"):
                lifecycle.start_all()
        stop_frontend.assert_not_called()
        stop_backend.assert_not_called()
        launch.assert_not_called()

    def test_existing_unresponsive_frontend_is_not_closed_by_failed_start(self):
        backend = {"running": True, "ready": True}
        with patch.object(lifecycle, "slicer_executable", return_value=Path("/apps/Slicer")), \
             patch.object(lifecycle, "backend_status", return_value=backend), \
             patch.object(lifecycle, "wait_ready", return_value=backend), \
             patch.object(lifecycle, "slicer_status", return_value={"running": True, "ready": False}), \
             patch.object(lifecycle, "stop_slicer") as stop_frontend, \
             patch.object(lifecycle.subprocess, "Popen") as launch, \
             patch.object(lifecycle.subprocess, "run") as stop_backend:
            with self.assertRaisesRegex(RuntimeError, "fresh heartbeat"):
                lifecycle.start_all()
        stop_frontend.assert_not_called()
        stop_backend.assert_not_called()
        launch.assert_not_called()

    def test_failed_new_frontend_is_cleaned_without_stopping_reused_backend(self):
        backend = {"running": True, "ready": True}
        with patch.object(lifecycle, "slicer_executable", return_value=Path("/apps/Slicer")), \
             patch.object(lifecycle, "backend_status", return_value=backend), \
             patch.object(lifecycle, "wait_ready", side_effect=[backend, RuntimeError("panel failed")]), \
             patch.object(lifecycle, "slicer_status", return_value={"running": False, "ready": False}), \
             patch.object(lifecycle, "process", return_value=self.proc), \
             patch.object(lifecycle, "stop_slicer") as stop_frontend, \
             patch.object(lifecycle.subprocess, "Popen", return_value=SimpleNamespace(pid=self.pid)) as launch, \
             patch.object(lifecycle.subprocess, "run") as stop_backend:
            with self.assertRaisesRegex(RuntimeError, "panel failed"):
                lifecycle.start_all()
        stop_frontend.assert_called_once_with()
        stop_backend.assert_not_called()
        launch.assert_called_once()

    def test_unresponsive_owned_frontend_can_stop_without_display_or_current_executable(self):
        self.save("linux_slicer.json", self.owner)
        with patch.dict(lifecycle.os.environ, {}, clear=True), \
             patch.object(lifecycle, "process", return_value=self.proc), \
             patch.object(lifecycle, "slicer_status", return_value={"running": True, "ready": False}), \
             patch.object(lifecycle, "group_members", side_effect=[[self.pid], []]), \
             patch.object(lifecycle, "slicer_executable", side_effect=AssertionError("Stopping must not launch/locate GUI")), \
             patch.object(lifecycle.os, "killpg") as terminate:
            lifecycle.stop_slicer()
        terminate.assert_called_once_with(self.pid, signal.SIGTERM)
        self.assertFalse((self.runtime / "linux_slicer.json").exists())

    def test_stop_refuses_reused_pid_or_unrelated_startup_script(self):
        for changed in ({"start_ticks": "9999"}, {"command": b"/apps/Slicer\0--python-script\0/unrelated/start.py"}):
            with self.subTest(changed=changed):
                self.save("linux_slicer.json", self.owner)
                proc = dict(self.proc, **changed)
                with patch.object(lifecycle, "process", return_value=proc), \
                     patch.object(lifecycle, "slicer_status", return_value={"running": True, "ready": False}), \
                     patch.object(lifecycle, "group_members", return_value=[self.pid]), \
                     patch.object(lifecycle.os, "killpg") as terminate:
                    with self.assertRaises(RuntimeError):
                        lifecycle.stop_slicer()
                terminate.assert_not_called()
                self.assertTrue((self.runtime / "linux_slicer.json").exists())


@unittest.skipUnless(lifecycle is not None, "Linux readiness uses /proc")
class LaunchFeedbackFreshnessTest(unittest.TestCase):
    """Exercise the actual checker with a fake /proc identity and JSON files."""
    CHECKER = Path(__file__).with_name("check_simulation.py").resolve()

    def check_state(self, timestamp):
        runtime = self.CHECKER.parent.parent / "runtime"
        fields = ["S", "1", "8100"] + ["0"] * 16 + ["1000"]
        texts = {
            str(runtime / "linux_launch.json"): json.dumps({"pid": 8100, "start_ticks": "1000", "started_at": 99.5}),
            "/proc/8100/stat": "8100 (ur12e launch) " + " ".join(fields),
            str(runtime / "state.json"): json.dumps({"timestamp": timestamp,
                "source": "ros2_mock_hardware", "ready": True, "feedback_valid": True,
                "request_id": None, "status": "idle"}),
        }

        def read_text(path, *args, **kwargs):
            return texts[str(path)]

        output = io.StringIO()
        with patch.object(Path, "read_text", new=read_text), \
             patch.object(Path, "read_bytes", return_value=b"python3\0ur12e_slicer\0mock.launch.py"), \
             patch("os.getpgid", return_value=8100), \
             patch("time.time", return_value=100.0), \
             contextlib.redirect_stdout(output):
            runpy.run_path(str(self.CHECKER), run_name="__main__")
        return json.loads(output.getvalue())

    def test_recent_feedback_from_before_current_launch_is_not_ready(self):
        result = self.check_state(99.4)
        self.assertTrue(result["running"])
        self.assertFalse(result["ready"])

    def test_fresh_feedback_from_current_launch_is_ready(self):
        self.assertTrue(self.check_state(99.8)["ready"])


if __name__ == "__main__":
    unittest.main()
