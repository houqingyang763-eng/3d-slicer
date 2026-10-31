"""Offline refusal checks; these tests never import rclpy or send ROS goals."""
import copy
import errno
import importlib.util
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

import validate_ursim as subject


KEY = "123456789abc"
NAME = "ur12e-slicer-ursim-" + KEY
LABELS = {"ur12e_slicer.task": "ursim", "ur12e_slicer.project_key": KEY}


def docker_fixture():
    container = {"Id": "container-id", "Name": "/" + NAME,
                 "Config": {"Labels": dict(LABELS),
                            "Image": "universalrobots/ursim_e-series@sha256:" + "a" * 64,
                            "Env": ["ROBOT_MODEL=UR12"]},
                 "State": {"Running": True}, "NetworkSettings": {"Networks": {
                     NAME: {"IPAddress": "192.168.62.2", "NetworkID": "network-id"}}}}
    network = {"Id": "network-id", "Name": NAME, "Labels": dict(LABELS),
               "Internal": True, "Driver": "bridge", "IPAM": {"Config": [
                   {"Subnet": "192.168.62.0/24", "Gateway": "192.168.62.1"}]},
               "Containers": {"container-id": {"Name": NAME, "IPv4Address": "192.168.62.2/24"}}}
    return container, network


def state_fixture():
    pose = {"frame_id": "base", "position_m": [0.2, -0.3, 0.4], "orientation_xyzw": [0, 0, 0, 1]}
    return {"source": "ros2_ursim", "schema_version": 1, "timestamp": 100.0,
            "feedback_valid": True, "ready": True, "joint_state_age_s": 0.01, "tf_age_s": 0.02,
            "joint_names": list(subject.JOINTS), "joint_positions_rad": list(subject.HOME),
            "needle_collision_ready": True, "tcp_pose": pose,
            "link_transforms": {name: copy.deepcopy(pose) for name in subject.REQUIRED_LINKS}}


class DockerIdentityTests(unittest.TestCase):
    def test_official_digest_and_isolated_task_endpoint(self):
        subject.check_docker_identity(*docker_fixture(), NAME, NAME, KEY)

    def test_refuses_unrelated_or_reachable_network_and_wrong_controller(self):
        mutations = {
            "wrong project": lambda c, n: c["Config"]["Labels"].update({"ur12e_slicer.project_key": "other"}),
            "unlabelled network": lambda c, n: n.update({"Labels": {}}),
            "external network": lambda c, n: n.update({"Internal": False}),
            "wrong robot IP": lambda c, n: c["NetworkSettings"]["Networks"][NAME].update({"IPAddress": "192.168.1.10"}),
            "wrong network ID": lambda c, n: c["NetworkSettings"]["Networks"][NAME].update({"NetworkID": "other"}),
            "extra network": lambda c, n: c["NetworkSettings"]["Networks"].update({"bridge": {}}),
            "extra member": lambda c, n: n["Containers"].update({"unrelated": {"Name": "other"}}),
            "wrong subnet": lambda c, n: n["IPAM"]["Config"][0].update({"Subnet": "192.168.1.0/24"}),
            "stopped container": lambda c, n: c["State"].update({"Running": False}),
            "wrong model": lambda c, n: c["Config"].update({"Env": ["ROBOT_MODEL=UR5"]}),
            "untrusted image": lambda c, n: c["Config"].update({"Image": "someone/ursim:latest"}),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                container, network = docker_fixture()
                mutate(container, network)
                with self.assertRaises(RuntimeError):
                    subject.check_docker_identity(container, network, NAME, NAME, KEY)

    def test_refuses_names_from_another_task(self):
        with self.assertRaises(RuntimeError):
            subject.check_docker_identity(*docker_fixture(), "another", NAME, KEY)


class RosIdentityAndStateTests(unittest.TestCase):
    def test_driver_description_must_target_the_verified_ursim_address(self):
        def description(plugin="ur_robot_driver/URPositionHardwareInterface", ip="192.168.62.2"):
            return f'<robot name="ur"><ros2_control name="UR"><hardware><plugin>{plugin}</plugin><param name="robot_ip">{ip}</param></hardware></ros2_control></robot>'
        subject.check_robot_description(description())
        for xml in (description(ip="192.168.1.10"), description(plugin="mock_components/GenericSystem"),
                    '<robot name="ur"/>', description().replace('</robot>', description() + '</robot>')):
            with self.subTest(xml=xml), self.assertRaises(RuntimeError):
                subject.check_robot_description(xml)

    def test_fresh_ready_ursim_state(self):
        subject.check_state(state_fixture(), 100.0, 100.1)

    def test_mock_stale_invalid_or_inconsistent_feedback_cannot_authorize(self):
        mutations = {
            "mock source": lambda s: s.update(source="ros2_mock_hardware"),
            "not ready": lambda s: s.update(ready=False),
            "invalid feedback": lambda s: s.update(feedback_valid=False),
            "stale whole state": lambda s: s.update(timestamp=98.0),
            "future state": lambda s: s.update(timestamp=101.0),
            "boolean timestamp": lambda s: s.update(timestamp=True),
            "old joints": lambda s: s.update(joint_state_age_s=1.01),
            "nonfinite TF age": lambda s: s.update(tf_age_s=float("nan")),
            "duplicate joint": lambda s: s["joint_names"].__setitem__(0, s["joint_names"][1]),
            "missing needle": lambda s: s.update(needle_collision_ready=False),
            "inconsistent TCP": lambda s: s["tcp_pose"]["position_m"].__setitem__(0, 0.3),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                state = state_fixture()
                mutate(state)
                with self.assertRaises((RuntimeError, AssertionError)):
                    subject.check_state(state, 100.0, 100.1)
        with self.assertRaises(RuntimeError):
            subject.check_state(state_fixture(), 98.0, 100.1)

    def test_linux_environment_refuses_root_or_another_ros_domain(self):
        with patch.object(subject.platform, "system", return_value="Linux"), \
                patch.object(subject.os, "geteuid", return_value=1000, create=True), \
                patch.dict(subject.os.environ, {"ROS_DOMAIN_ID": "62", "ROS_LOCALHOST_ONLY": "1"}):
            subject.check_environment()
            with patch.object(subject.os, "geteuid", return_value=0):
                with self.assertRaises(RuntimeError):
                    subject.check_environment()
            with patch.dict(subject.os.environ, {"ROS_DOMAIN_ID": "0"}):
                with self.assertRaises(RuntimeError):
                    subject.check_environment()


class FailureEvidenceTests(unittest.TestCase):
    def test_late_accepted_goal_is_canceled_and_terminal_status_confirmed(self):
        events = []
        terminal = SimpleNamespace(status=5, result=SimpleNamespace(error_code=0, error_string="canceled"))
        canceled = SimpleNamespace(return_code=0, goals_canceling=["owned-goal"])
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: terminal,
                                 cancel_goal_async=lambda: canceled)
        session = subject.RosSession.__new__(subject.RosSession)
        session.pending_goal = handle
        session.active_goal = None
        session.active_result = None
        session.wait_future = lambda future, timeout, label: future
        session.validation = SimpleNamespace(log=lambda event, detail: events.append((event, detail)))
        session.cancel_owned_goal()
        self.assertIsNone(session.pending_goal)
        self.assertIsNone(session.active_goal)
        self.assertEqual(events[-1], ("cancel_goal_terminal", {"action_status": 5,
            "controller_error_code": 0, "controller_error_string": "canceled"}))

    def test_cancellation_acknowledgement_without_terminal_result_is_failure(self):
        canceled = SimpleNamespace(return_code=0, goals_canceling=["owned-goal"])
        handle = SimpleNamespace(cancel_goal_async=lambda: canceled)
        session = subject.RosSession.__new__(subject.RosSession)
        session.pending_goal = None
        session.active_goal = handle
        session.active_result = object()
        session.validation = SimpleNamespace(log=lambda *args: None)

        def wait(future, timeout, label):
            if future is canceled:
                return canceled
            raise RuntimeError("no terminal feedback")
        session.wait_future = wait
        with self.assertRaisesRegex(RuntimeError, "confirm termination"):
            session.cancel_owned_goal()
        self.assertIs(session.active_goal, handle)

    def test_unsupported_host_leaves_failed_summary_without_starting_ros_or_docker(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / "runtime"
            runtime.mkdir()
            output = Path(directory) / "results"
            with patch.object(subject.platform, "system", return_value="Windows"), \
                    patch.object(subject.subprocess, "run") as process:
                code = subject.main(["--runtime", str(runtime), "--output", str(output),
                                     "--container", NAME, "--network", NAME])
            self.assertEqual(code, 1)
            process.assert_not_called()
            summaries = list(output.glob("*/summary.json"))
            self.assertEqual(len(summaries), 1)
            summary = json.loads(summaries[0].read_text(encoding="utf-8"))
            self.assertFalse(summary["passed"])
            self.assertIn("requires Linux", summary["error"])
            self.assertEqual(summary["cases"], [])
            self.assertFalse((runtime / "validation.lock").exists())


class SlicerSourceGateTests(unittest.TestCase):
    """Exercise the real acceptance source gates with inert GUI dependencies."""
    def setUp(self):
        class Timer:
            def __init__(self):
                self.active = False
            def setInterval(self, *args):
                pass
            def connect(self, *args):
                pass
            def stop(self):
                self.active = False
            def isActive(self):
                return self.active

        self.sent = []
        self.panel = SimpleNamespace(state=state_fixture(), recording=None, poll=lambda: None,
                                     capture=lambda path: None, send=lambda command: self.sent.append(command))
        self.slicer = SimpleNamespace(ur12eSimulation=self.panel, app=SimpleNamespace(applicationVersion="test-only"))
        spec = importlib.util.spec_from_file_location("slicer_acceptance_gate_test", Path(__file__).with_name("slicer_acceptance.py"))
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"numpy": SimpleNamespace(), "qt": SimpleNamespace(QTimer=Timer),
                                      "slicer": self.slicer, "vtk": SimpleNamespace()}):
            spec.loader.exec_module(self.module)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.module.PROJECT = Path(self.folder.name)

    def test_mock_defaults_retain_original_evidence_and_status_paths(self):
        gate = self.module.SlicerAcceptance()
        self.assertEqual(gate.expected_source, "ros2_mock_hardware")
        self.assertEqual(gate.report["evidence_type"], "real_slicer_ros2_moveit_mock_hardware_loop")
        self.assertEqual(gate.source_details["feedback_check"], "genuine_ros2_mock_feedback")
        self.assertEqual(gate.folder.parent.name, "slicer_acceptance")
        self.assertEqual(gate.status_path.name, "slicer_acceptance_status.json")

    def test_only_two_explicit_sources_are_supported(self):
        with self.assertRaises(ValueError):
            self.module.SlicerAcceptance(expected_source="physical_robot")

    def test_ursim_start_rejects_mock_before_target_controls_or_sender(self):
        gate = self.module.SlicerAcceptance(expected_source="ros2_ursim")
        self.panel.state["source"] = "ros2_mock_hardware"
        with self.assertRaisesRegex(RuntimeError, "expected source=ros2_ursim"):
            gate.start()
        self.assertEqual(self.sent, [])

    def test_source_swap_during_tick_fails_before_a_new_plan(self):
        for expected, actual in (("ros2_ursim", "ros2_mock_hardware"), ("ros2_mock_hardware", "ros2_ursim")):
            with self.subTest(expected=expected):
                gate = self.module.SlicerAcceptance(expected_source=expected)
                gate.stage = "target_hold"
                gate.stage_started = subject.time.monotonic() - 3
                gate.timer.active = True
                self.panel.state["source"] = actual
                gate.tick()
                self.assertEqual(gate.stage, "failed")
                self.assertFalse(gate.timer.isActive())
                self.assertEqual(self.sent, [])
                report = json.loads((gate.folder / "results.json").read_text(encoding="utf-8"))
                self.assertFalse(report["all_passed"])
                self.assertEqual(report["source"], expected)
                self.assertEqual(gate.samples[-1]["state"]["source"], actual)

    def test_ursim_report_is_explicitly_virtual_and_uses_separate_evidence(self):
        gate = self.module.SlicerAcceptance(expected_source="ros2_ursim")
        self.panel.state["timestamp"] = subject.time.time()
        gate.tick()
        self.assertEqual(gate.stage, "prepared")
        self.assertEqual(gate.report["evidence_type"], "real_slicer_ros2_moveit_ursim_virtual_controller_loop")
        self.assertTrue(gate.report["simulation_only"])
        self.assertFalse(gate.report["actual_robot_connected"])
        self.assertIn("No physical robot", gate.report["scope"])
        self.assertEqual(gate.source_details["feedback_check"], "genuine_ros2_ursim_feedback")
        self.assertEqual(gate.status_path.name, "ursim_slicer_acceptance_status.json")
        self.assertEqual(gate.folder.parent.name, "ursim_slicer_acceptance")

    def test_ursim_capture_finishes_before_hold_clock_starts(self):
        for frame_count in (7, 22):
            with self.subTest(frame_count=frame_count):
                gate = self.module.SlicerAcceptance(expected_source="ros2_ursim")
                gate.stage = "executing"
                gate.stage_started = 50.0
                gate.report["execute_request"] = {"request_id": "owned"}
                self.panel.state.update(source="ros2_ursim", request_id="owned", status="succeeded",
                                        timestamp=subject.time.time())
                self.panel.recording = gate.folder / "frames"
                self.panel.recorded_frames = list(range(frame_count))
                clock = [100.0]
                stopped = []

                def stop_capture():
                    stopped.append(clock[0])
                    # Capturing/closing can take time; it must not consume the
                    # hold window which starts after this operation returns.
                    clock[0] += 0.3
                    self.panel.recording = None
                self.panel.toggle_recording = stop_capture
                with patch.object(self.module.time, "monotonic", side_effect=lambda: clock[0]):
                    gate.tick()
                self.assertEqual(gate.stage, "feedback_hold")
                self.assertEqual(stopped, [100.0])
                self.assertEqual(gate.stage_started, 100.3)
                self.assertTrue(gate.report["recording_ended_before_feedback_hold"])
                self.assertEqual(gate.report["recording_frame_count"], frame_count)
                self.assertEqual(len(self.panel.recorded_frames), frame_count)
                self.assertIsNone(self.panel.recording)

    def test_mock_keeps_recording_during_its_existing_hold_behavior(self):
        gate = self.module.SlicerAcceptance()
        gate.stage = "executing"
        gate.report["execute_request"] = {"request_id": "owned"}
        self.panel.state.update(source="ros2_mock_hardware", request_id="owned", status="succeeded",
                                timestamp=subject.time.time())
        self.panel.recording = gate.folder / "frames"
        self.panel.toggle_recording = Mock()
        gate.tick()
        self.assertEqual(gate.stage, "feedback_hold")
        self.panel.toggle_recording.assert_not_called()
        self.assertIsNotNone(self.panel.recording)
        self.assertNotIn("recording_ended_before_feedback_hold", gate.report)


class StorageVisibilityRetryTests(unittest.TestCase):
    def setUp(self):
        self.clock = 0.0
        self.sleeps = []
        self.events = []
        self.gate = subject.UrsimValidation.__new__(subject.UrsimValidation)
        self.gate.ros = None
        self.gate.state_path = Mock()
        self.gate.state_path.stat.return_value = SimpleNamespace(st_mtime=100.0)
        self.gate.args = SimpleNamespace(max_feedback_age=1.0)
        self.gate.summary = {}
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.gate.output = Path(self.folder.name)
        self.gate.last_trace_timestamp = None
        self.gate.trace = io.StringIO()
        self.gate.log = lambda event, detail: self.events.append((event, detail))
        self.raw = json.dumps(state_fixture())
        self.patches = [patch.object(subject.time, "monotonic", side_effect=lambda: self.clock),
                        patch.object(subject.time, "time", return_value=100.1),
                        patch.object(subject.time, "sleep", side_effect=self.sleep)]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.clock += seconds

    def test_transient_enodata_recovers_without_changing_freshness(self):
        self.gate.state_path.read_text.side_effect = [OSError(getattr(errno, "ENODATA", 61), "No data available"), self.raw]
        state = self.gate.read_state()
        self.assertEqual(state["source"], "ros2_ursim")
        self.assertEqual(self.gate.state_path.read_text.call_count, 2)
        self.assertLessEqual(sum(self.sleeps), 0.25)
        self.assertEqual(self.events[0][0], "state_storage_retry")

    def test_stat_visibility_error_restarts_the_whole_read(self):
        self.gate.state_path.read_text.return_value = self.raw
        self.gate.state_path.stat.side_effect = [OSError(errno.ENOENT, "replacement visibility"), SimpleNamespace(st_mtime=100.0)]
        self.gate.read_state()
        self.assertEqual(self.gate.state_path.read_text.call_count, 2)

    def test_persistent_enodata_stops_at_total_250ms_budget(self):
        self.gate.state_path.read_text.side_effect = OSError(getattr(errno, "ENODATA", 61), "persistent visibility failure")
        with self.assertRaises(OSError):
            self.gate.read_state()
        self.assertLessEqual(sum(self.sleeps), 0.25)
        self.assertGreaterEqual(self.gate.state_path.read_text.call_count, 2)
        self.assertLessEqual(self.gate.state_path.read_text.call_count, 26)
        self.assertEqual(self.gate.trace.getvalue(), "")

    def test_non_visibility_io_error_is_not_retried(self):
        self.gate.state_path.read_text.side_effect = OSError(errno.EIO, "I/O failure")
        with self.assertRaises(OSError):
            self.gate.read_state()
        self.assertEqual(self.gate.state_path.read_text.call_count, 1)
        self.assertEqual(self.sleeps, [])

    def test_invalid_and_stale_states_after_recovery_fail_without_further_retry(self):
        for change in ({"source": "ros2_mock_hardware"}, {"timestamp": 98.0}, {"feedback_valid": False}):
            with self.subTest(change=change):
                state = state_fixture()
                state.update(change)
                self.gate.state_path.read_text.reset_mock()
                self.gate.state_path.read_text.side_effect = [OSError(errno.EACCES, "sharing window"), json.dumps(state), self.raw]
                with self.assertRaises(RuntimeError):
                    self.gate.read_state()
                self.assertEqual(self.gate.state_path.read_text.call_count, 2)

    def test_malformed_json_is_not_a_storage_retry(self):
        self.gate.state_path.read_text.return_value = "{incomplete"
        with self.assertRaises(json.JSONDecodeError):
            self.gate.read_state()
        self.assertEqual(self.gate.state_path.read_text.call_count, 1)
        self.assertEqual(self.sleeps, [])

    def test_first_rejected_state_is_saved_once_with_clock_diagnostics(self):
        state = state_fixture()
        state["timestamp"] = 98.0
        self.gate.state_path.read_text.return_value = json.dumps(state)
        with self.assertRaises(RuntimeError):
            self.gate.read_state()
        path = self.gate.output / "rejected_state.json"
        first_bytes = path.read_bytes()
        recorded = json.loads(first_bytes)
        self.assertEqual(recorded["state"], state)
        self.assertEqual(json.loads(recorded["raw_state_json"]), state)
        self.assertAlmostEqual(recorded["now"], 100.1)
        self.assertAlmostEqual(recorded["state_age_s"], 2.1)
        self.assertAlmostEqual(recorded["file_age_s"], 0.1)
        self.assertEqual(recorded["read_elapsed_monotonic_s"], 0.0)
        self.assertEqual(self.gate.summary["first_rejected_state"], recorded)
        state["timestamp"] = 102.0
        self.gate.state_path.read_text.return_value = json.dumps(state)
        with self.assertRaises(RuntimeError):
            self.gate.read_state()
        self.assertEqual(path.read_bytes(), first_bytes)
        self.assertEqual(self.gate.state_path.read_text.call_count, 2)
        self.assertEqual(self.sleeps, [])


if __name__ == "__main__":
    unittest.main()
