"""Offline URSim ownership/readiness checks; Docker, sockets and signals are mocked."""
import json
import io
import os
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import ursim_lifecycle as lifecycle


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


class UrSimLifecycleTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="ursim lifecycle ")
        self.addCleanup(temporary.cleanup)
        self.project = Path(temporary.name) / "ur12e_slicer"
        self.runtime = self.project / "runtime"
        self.cache = Path(temporary.name) / "cache"
        self.runtime.mkdir(parents=True)
        self.shared = SimpleNamespace(process=MagicMock(return_value=None),
                                      group_members=MagicMock(return_value=[]),
                                      slicer_status=MagicMock(return_value={"running": False, "ready": False}),
                                      write_json=save, stamp=lambda: "test-run")
        patched = patch.multiple(lifecycle, PROJECT=self.project, REPO=self.project.parent,
                                 RUNTIME=self.runtime, BACKEND_OWNER=self.runtime / "ursim_backend.json",
                                 CONTAINER_OWNER=self.runtime / "ursim_container.json",
                                 PROXY_OWNER=self.runtime / "ursim_gui_proxy.json")
        patched.start()
        self.addCleanup(patched.stop)
        shared = patch.object(lifecycle, "common", return_value=self.shared)
        shared.start()
        self.addCleanup(shared.stop)
        cache = patch.object(lifecycle, "cache_root", return_value=self.cache)
        cache.start()
        self.addCleanup(cache.stop)

    def network(self):
        return {"Id": "network-id", "Name": lifecycle.NAME, "Driver": "bridge", "Internal": True,
                "Labels": dict(lifecycle.LABELS), "IPAM": {"Config": [{"Subnet": lifecycle.SUBNET,
                "Gateway": lifecycle.GATEWAY}]}, "Containers": {}}

    def image(self):
        return {"Id": "image-id", "RepoDigests": [lifecycle.IMAGE]}

    def container(self):
        return {"Id": "container-id", "Name": "/" + lifecycle.NAME, "Image": "image-id",
                "Config": {"Labels": dict(lifecycle.LABELS), "Image": lifecycle.IMAGE, "Env": ["ROBOT_MODEL=UR12"]},
                "HostConfig": {"Privileged": False, "NetworkMode": lifecycle.NAME,
                    "PortBindings": {"6080/tcp": [{"HostIp": "127.0.0.1", "HostPort": "6080"}]}},
                "State": {"Running": True}, "NetworkSettings": {"Networks": {lifecycle.NAME: {
                    "NetworkID": "network-id", "IPAMConfig": {"IPv4Address": lifecycle.ROBOT_IP},
                    "IPAddress": lifecycle.ROBOT_IP}}},
                "Mounts": [{"Type": "bind", "Destination": target, "Source": str(self.cache / folder)}
                           for target, folder in (("/ursim/programs", "programs"), ("/urcaps", "urcaps"))]}

    def backend(self, source="ros2_ursim", timestamp=101.0):
        save(lifecycle.BACKEND_OWNER, {"pid": 8100, "group": 8100, "start_ticks": "77", "started_at": 100.0})
        save(lifecycle.control_runtime() / "state.json", {"timestamp": timestamp, "source": source,
                                         "ready": True, "feedback_valid": True, "status": "idle"})
        self.shared.process.return_value = {"pid": 8100, "group": 8100, "start_ticks": "77", "state": "S",
            "command": b"ros2\0launch\0ur12e_slicer_bridge\0ursim.launch.py\0runtime_dir:=" + str(lifecycle.control_runtime()).encode()}
        self.shared.group_members.return_value = [8100]

    def test_only_fixed_internal_network_with_task_labels_is_accepted(self):
        self.assertEqual(lifecycle.validate_network(self.network())["Id"], "network-id")
        for mutation in (lambda value: value.update(Internal=False),
                         lambda value: value["Labels"].update({"ur12e_slicer.project_key": "other"}),
                         lambda value: value["IPAM"]["Config"][0].update(Gateway="192.168.1.1"),
                         lambda value: value["Containers"].update({"foreign": {"Name": "another-task"}})):
            value = self.network()
            mutation(value)
            with self.assertRaises(RuntimeError):
                lifecycle.validate_network(value)

    def test_container_must_match_image_model_address_network_ports_and_mounts(self):
        lifecycle.validate_container(self.container(), self.network(), self.image())
        changes = (
            lambda value: value["Config"].update(Image="universalrobots/ursim_e-series:latest"),
            lambda value: value["Config"].update(Env=["ROBOT_MODEL=UR5"]),
            lambda value: value["Config"]["Labels"].update({"ur12e_slicer.task": "other"}),
            lambda value: value["HostConfig"].update(Privileged=True),
            lambda value: value["HostConfig"]["PortBindings"]["6080/tcp"][0].update(HostIp="0.0.0.0"),
            lambda value: value["NetworkSettings"]["Networks"][lifecycle.NAME].update(IPAddress="192.168.1.20"),
            lambda value: value["NetworkSettings"]["Networks"].update({"bridge": {}}),
            lambda value: value["Mounts"][0].update(Source="/other-task/programs"),
        )
        for change in changes:
            value = self.container()
            change(value)
            with self.assertRaises(RuntimeError):
                lifecycle.validate_container(value, self.network(), self.image())

    def test_image_without_expected_digest_is_rejected(self):
        with self.assertRaises(RuntimeError):
            lifecycle.validate_container(self.container(), self.network(), {"Id": "image-id", "RepoDigests": []})

    def test_preparation_without_assets_never_pulls_or_creates(self):
        docker = MagicMock()
        with self.assertRaises(RuntimeError):
            lifecycle.prepare_container(docker)
        docker.call.assert_not_called()

    def test_backend_needs_fresh_ursim_feedback_and_exact_process_identity(self):
        self.backend()
        with patch.object(lifecycle.time, "time", return_value=101.5):
            self.assertTrue(lifecycle.backend_status()["ready"])
            self.backend(source="ros2_mock_hardware")
            self.assertFalse(lifecycle.backend_status()["ready"])
            self.backend(timestamp=99.0)
            self.assertFalse(lifecycle.backend_status()["ready"])
            self.backend()
            self.shared.process.return_value["start_ticks"] = "reused"
            self.assertFalse(lifecycle.backend_status()["ready"])

    def test_stop_refuses_reused_pid_or_changed_command_and_preserves_record(self):
        for key, wrong in (("start_ticks", "reused"), ("group", 9000), ("command", b"another-task")):
            self.backend()
            self.shared.process.return_value[key] = wrong
            with patch.object(lifecycle.os, "kill") as send_signal, self.assertRaises(RuntimeError):
                lifecycle.stop_backend()
            send_signal.assert_not_called()
            self.assertTrue(lifecycle.BACKEND_OWNER.exists())

    def test_successful_backend_stop_keeps_owner_history(self):
        self.backend()
        self.shared.group_members.return_value = []
        with patch.object(lifecycle.os, "kill") as send_signal:
            lifecycle.stop_backend()
        send_signal.assert_called_once_with(8100, lifecycle.signal.SIGINT)
        self.assertIn("stopped_at", lifecycle.read_json(lifecycle.BACKEND_OWNER))

    def test_live_mock_blocks_ursim_start_and_stop_before_side_effects(self):
        with patch.object(lifecycle, "mock_is_live", return_value=True), \
                patch.object(lifecycle, "Docker") as docker, \
                patch.object(lifecycle, "stop_frontend") as stop_frontend:
            with self.assertRaises(RuntimeError):
                lifecycle.start_all(backend_only=True)
            with self.assertRaises(RuntimeError):
                lifecycle.stop_all()
        docker.assert_not_called()
        stop_frontend.assert_not_called()

    def test_container_replacement_is_not_stopped(self):
        save(lifecycle.CONTAINER_OWNER, {"container_id": "old-container", "network_id": "network-id",
             "docker_endpoint": "unix:///local.sock", "project_key": lifecycle.PROJECT_KEY})
        docker = MagicMock(endpoint="unix:///local.sock")
        docker.inspect.return_value = self.network()
        with patch.object(lifecycle, "verified_container", return_value=self.container()), self.assertRaises(RuntimeError):
            lifecycle.stop_container(docker)
        docker.call.assert_not_called()
        self.assertEqual(lifecycle.read_json(lifecycle.CONTAINER_OWNER)["container_id"], "old-container")

    def test_status_does_not_create_files_or_mutate_docker(self):
        docker = MagicMock()
        docker.inspect.return_value = None
        before = list(self.runtime.iterdir())
        with patch.object(lifecycle, "Docker", return_value=docker):
            result = lifecycle.status()
        self.assertFalse(result["ready"])
        docker.call.assert_not_called()
        self.assertEqual(list(self.runtime.iterdir()), before)

    def test_simulator_only_starts_no_ros_slicer_or_motion_commands(self):
        with patch.object(lifecycle, "mock_is_live", return_value=False), \
                patch.object(lifecycle, "Docker"), \
                patch.object(lifecycle, "prepare_container", return_value=(self.container(), False)), \
                patch.object(lifecycle, "dashboard", return_value="Robotmode: POWER_OFF") as dashboard, \
                patch.object(lifecycle, "start_proxy", return_value=({"ready": True}, False)), \
                patch.object(lifecycle, "build_workspace") as build, \
                patch.object(lifecycle, "launch_backend") as launch, \
                patch.object(lifecycle, "start_frontend") as frontend, \
                patch("sys.stdout", new=io.StringIO()):
            result = lifecycle.start_all(simulator_only=True)
        self.assertTrue(result["simulator_only"])
        self.assertEqual([call[0][1] for call in dashboard.call_args_list], ["robotmode"])
        build.assert_not_called()
        launch.assert_not_called()
        frontend.assert_not_called()

    def test_reused_proxy_pid_is_not_signalled(self):
        save(lifecycle.PROXY_OWNER, {"pid": 9100, "group": 9100, "start_ticks": "77", "ready": True,
             "listen": "127.0.0.1:6080", "target": lifecycle.ROBOT_IP + ":6080"})
        self.shared.process.return_value = {"pid": 9100, "group": 9100, "start_ticks": "changed", "state": "S",
             "command": str(self.project / "bootstrap/ursim_gui_proxy.py").encode()}
        self.shared.group_members.return_value = [9100]
        with patch.object(lifecycle.os, "kill") as send_signal, self.assertRaises(RuntimeError):
            lifecycle.stop_proxy()
        send_signal.assert_not_called()
        self.assertTrue(lifecycle.PROXY_OWNER.exists())

    def test_zero_validator_exit_without_fresh_passing_ursim_report_is_failure(self):
        args = SimpleNamespace(no_slicer=True, stop_after=False)
        with patch.object(lifecycle, "start_all"), \
                patch.object(lifecycle.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as run, \
                patch("sys.stdout", new=io.StringIO()), self.assertRaises(RuntimeError):
            lifecycle.run_checks(args)
        command = run.call_args[0][0]
        self.assertEqual(command[:2], ["bash", "-c"])
        self.assertIn("source /opt/ros/jazzy/setup.bash", command[2])
        self.assertIn('source "$1/install/setup.bash"', command[2])
        self.assertIn('exec "$@"', command[2])
        self.assertEqual(command[5], "/usr/bin/python3")
        self.assertEqual(command[6], str(self.project / "validation/validate_ursim.py"))
        self.assertEqual(command[command.index("--runtime") + 1], str(lifecycle.control_runtime()))
        self.assertEqual(command[command.index("--output") + 1], str(self.cache / "validation/test-run"))
        self.assertEqual(run.call_args[1]["env"]["ROS_DOMAIN_ID"], "62")
        self.assertEqual(run.call_args[1]["env"]["ROS_LOCALHOST_ONLY"], "1")
        report = lifecycle.read_json(self.project / "output/ursim_validation/test-run/lifecycle_summary.json")
        self.assertFalse(report["passed"])
        self.assertEqual(report["validation_exit_code"], 0)

    def acceptance_sender(self, changes=None, stage="complete", name="run_new"):
        acceptance = self.project / "output/ursim_slicer_acceptance" / name
        report = {"source": "ros2_ursim", "expected_source": "ros2_ursim",
                  "evidence_type": "real_slicer_ros2_moveit_ursim_virtual_controller_loop",
                  "simulation_only": True, "actual_robot_connected": False, "completed_at": 101.0,
                  "all_passed": True, "checks": {"observed_joint_motion": True, "scene_saved": True},
                  "output_directory": str(acceptance.resolve())}
        report.update(changes or {})
        entry = self.project / "validation/start_ursim_slicer_acceptance.py"
        entry.parent.mkdir(parents=True, exist_ok=True)
        entry.write_text("# test acceptance start\n", encoding="utf-8")
        calls = []

        def send(code, timeout):
            calls.append(code)
            if len(calls) > 1:
                return {"ok": True, "result": {"owned_acceptance_stopped": True}}
            save(acceptance / "results.json", report)
            save(self.project / "output/ursim_slicer_acceptance_status.json", {
                "timestamp": 101.0, "stage": stage, "expected_source": "ros2_ursim",
                "output_directory": str(acceptance.resolve())})
            return {"ok": True, "result": {"stage": "target_hold", "output_directory": str(acceptance.resolve())}}
        return MagicMock(side_effect=send), acceptance

    def test_slicer_requires_new_complete_ursim_report_and_records_path(self):
        sender, acceptance = self.acceptance_sender()
        result = {}
        folder = self.project / "output/lifecycle/run"
        with patch.object(lifecycle.time, "time", return_value=100.0):
            path = lifecycle.check_slicer_acceptance(folder, result, sender=sender)
        self.assertTrue(result["slicer_passed"])
        self.assertEqual(path, acceptance.resolve() / "results.json")
        self.assertEqual(result["slicer_result"], str(path.relative_to(self.project.resolve())))
        sender.assert_called_once()

    def test_slicer_wrong_source_scope_or_failed_checks_are_retained_as_failure(self):
        changes = ({"source": "ros2_mock_hardware"}, {"expected_source": "ros2_mock_hardware"},
                   {"evidence_type": "gui_opened_only"}, {"all_passed": False},
                   {"checks": {"observed_joint_motion": False}}, {"completed_at": 99.0})
        for index, change in enumerate(changes):
            with self.subTest(change=change):
                sender, acceptance = self.acceptance_sender(change, name="run_case_" + str(index))
                result = {}
                folder = self.project / "output/lifecycle" / ("case_" + str(index))
                with patch.object(lifecycle.time, "time", return_value=100.0), self.assertRaises(RuntimeError):
                    lifecycle.check_slicer_acceptance(folder, result, sender=sender)
                self.assertFalse(result["slicer_passed"])
                self.assertEqual(lifecycle.read_json(acceptance / "results.json").get(next(iter(change))), next(iter(change.values())))
                self.assertEqual(sender.call_count, 2)

    def test_slicer_timeout_stops_only_returned_acceptance_without_overwriting_report(self):
        sender, acceptance = self.acceptance_sender(stage="executing")
        result = {}
        with patch.object(lifecycle.time, "time", return_value=100.0), self.assertRaises(RuntimeError):
            lifecycle.check_slicer_acceptance(self.project / "output/lifecycle/timeout", result, timeout=0, sender=sender)
        self.assertFalse(result["slicer_passed"])
        self.assertEqual(sender.call_count, 2)
        self.assertIn(repr(str(acceptance.resolve())), sender.call_args_list[1][0][0])
        self.assertTrue(lifecycle.read_json(acceptance / "results.json")["all_passed"])

    def test_full_runner_never_passes_basic_success_plus_failed_slicer(self):
        args = SimpleNamespace(no_slicer=False, stop_after=False)
        folder = self.project / "output/ursim_validation/test-run"
        def validator(command, **_kwargs):
            original = Path(command[command.index("--output") + 1])
            save(original / "basic/summary.json", {"passed": True, "source": "ros2_ursim",
                 "mode": "ursim_virtual_controller_basic_control"})
            return SimpleNamespace(returncode=0)
        with patch.object(lifecycle, "start_all"), \
                patch.object(lifecycle.subprocess, "run", side_effect=validator), \
                patch.object(lifecycle, "check_slicer_acceptance", side_effect=RuntimeError("Slicer checks failed")), \
                patch("sys.stdout", new=io.StringIO()), self.assertRaises(RuntimeError):
            lifecycle.run_checks(args)
        self.assertFalse(lifecycle.read_json(folder / "lifecycle_summary.json")["passed"])

    def test_control_runtime_is_cache_isolated_from_management_runtime(self):
        self.assertEqual(lifecycle.control_runtime(), (self.cache / "bridge").resolve())
        self.assertNotEqual(lifecycle.control_runtime(), self.runtime)
        self.backend()
        save(self.runtime / "state.json", {"source": "ros2_mock_hardware", "timestamp": 0})
        with patch.object(lifecycle.time, "time", return_value=101.5):
            self.assertTrue(lifecycle.backend_status()["ready"])

    def test_failed_basic_run_is_archived_from_cache_without_changing_report(self):
        args = SimpleNamespace(no_slicer=True, stop_after=False)
        original = self.cache / "validation/test-run"
        def validator(command, **_kwargs):
            self.assertEqual(Path(command[command.index("--output") + 1]), original)
            save(original / "failed_case/summary.json", {"passed": False, "source": "ros2_ursim", "error": "freshness failed"})
            (original / "failed_case/joint_trace.jsonl").write_text('{"sample":1}\n', encoding="utf-8")
            return SimpleNamespace(returncode=1)
        with patch.object(lifecycle, "start_all"), \
                patch.object(lifecycle.subprocess, "run", side_effect=validator), \
                patch("sys.stdout", new=io.StringIO()), self.assertRaises(RuntimeError):
            lifecycle.run_checks(args)
        published = self.project / "output/ursim_validation/test-run"
        self.assertEqual((published / "failed_case/summary.json").read_bytes(), (original / "failed_case/summary.json").read_bytes())
        self.assertTrue((original / "failed_case/joint_trace.jsonl").exists())
        self.assertTrue((published / "failed_case/joint_trace.jsonl").exists())
        report = lifecycle.read_json(published / "lifecycle_summary.json")
        self.assertFalse(report["passed"])
        self.assertEqual(report["original_validation_directory"], str(original))

    def test_archive_preserves_nested_binary_evidence_without_metadata_permissions(self):
        original = self.cache / "validation/original"
        published = self.project / "output/ursim_validation/archive"
        published.mkdir(parents=True)
        save(original / "run_case/summary.json", {"passed": False, "source": "ros2_ursim"})
        payload = b"\x00\xff\x80\nframe-data\x00"
        (original / "run_case/nested").mkdir()
        (original / "run_case/nested/frame.bin").write_bytes(payload)
        (original / "run_case/empty").mkdir()
        result = {}
        with patch.object(lifecycle.os, "utime", side_effect=PermissionError("DrvFS utime denied")) as utime, \
                patch.object(lifecycle.os, "chmod", side_effect=PermissionError("DrvFS chmod denied")) as chmod:
            lifecycle.archive_validation(original, published, result)
        utime.assert_not_called()
        chmod.assert_not_called()
        self.assertEqual((published / "run_case/nested/frame.bin").read_bytes(), payload)
        self.assertEqual((original / "run_case/nested/frame.bin").read_bytes(), payload)
        self.assertTrue((published / "run_case/empty").is_dir())
        self.assertEqual(result["integration_original_result"], str(original / "run_case/summary.json"))
        self.assertEqual(result["integration_result"], str((published / "run_case/summary.json").relative_to(self.project)))
        self.assertEqual(lifecycle.read_json(published / "archive_provenance.json")["original_validation_directory"], str(original))

    def test_archive_refuses_existing_destination_without_replacing_evidence(self):
        original = self.cache / "validation/original"
        published = self.project / "output/ursim_validation/archive"
        original.mkdir(parents=True)
        published.mkdir(parents=True)
        (original / "trace.jsonl").write_bytes(b"new evidence\n")
        (published / "trace.jsonl").write_bytes(b"existing evidence\n")
        with self.assertRaisesRegex(RuntimeError, "replace existing"):
            lifecycle.archive_validation(original, published, {})
        self.assertEqual((original / "trace.jsonl").read_bytes(), b"new evidence\n")
        self.assertEqual((published / "trace.jsonl").read_bytes(), b"existing evidence\n")
        self.assertFalse((published / "archive_provenance.json").exists())

    def test_archive_hash_mismatch_is_failure_and_keeps_original(self):
        original = self.cache / "validation/original"
        published = self.project / "output/ursim_validation/archive"
        original.mkdir(parents=True)
        published.mkdir(parents=True)
        (original / "trace.jsonl").write_bytes(b"original valid evidence\n")
        def corrupted_copy(_source, target):
            Path(target).write_bytes(b"damaged copy\n")
        with patch.object(lifecycle.shutil, "copyfile", side_effect=corrupted_copy), \
                self.assertRaisesRegex(RuntimeError, "hash mismatch"):
            lifecycle.archive_validation(original, published, {})
        self.assertEqual((original / "trace.jsonl").read_bytes(), b"original valid evidence\n")
        self.assertEqual((published / "trace.jsonl").read_bytes(), b"damaged copy\n")
        self.assertFalse((published / "archive_provenance.json").exists())

    def test_archive_rejects_source_symlink_without_following_it(self):
        original = self.cache / "validation/original"
        published = self.project / "output/ursim_validation/archive"
        original.mkdir(parents=True)
        published.mkdir(parents=True)
        outside = self.project.parent / "outside-evidence.json"
        outside.write_bytes(b"unrelated evidence\n")
        link = original / "linked.json"
        try:
            link.symlink_to(outside)
        except (OSError, NotImplementedError) as error:
            self.skipTest("This environment cannot create test symlinks: " + str(error))
        with self.assertRaisesRegex(RuntimeError, "symlink"):
            lifecycle.archive_validation(original, published, {})
        self.assertEqual(outside.read_bytes(), b"unrelated evidence\n")
        self.assertTrue(link.is_symlink())
        self.assertFalse((published / "linked.json").exists())

    def test_archive_rejects_dangling_target_symlink_without_creating_external_file(self):
        original = self.cache / "validation/original"
        published = self.project / "output/ursim_validation/archive"
        original.mkdir(parents=True)
        published.mkdir(parents=True)
        (original / "trace.jsonl").write_bytes(b"original valid evidence\n")
        outside = self.project.parent / "must-not-be-created.jsonl"
        link = published / "trace.jsonl"
        try:
            link.symlink_to(outside)
        except (OSError, NotImplementedError) as error:
            self.skipTest("This environment cannot create test symlinks: " + str(error))
        self.assertFalse(link.exists())
        self.assertTrue(link.is_symlink())
        with self.assertRaisesRegex(RuntimeError, "replace existing"):
            lifecycle.archive_validation(original, published, {})
        self.assertFalse(outside.exists())
        self.assertTrue(link.is_symlink())
        self.assertEqual((original / "trace.jsonl").read_bytes(), b"original valid evidence\n")


class DockerSocketTests(unittest.TestCase):
    def test_remote_context_is_rejected_without_docker_mutation(self):
        response = SimpleNamespace(returncode=0, stdout=json.dumps([
            {"Endpoints": {"docker": {"Host": "ssh://lab-robot-host"}}}]), stderr="")
        with patch.dict(os.environ, {}, clear=True), patch.object(lifecycle.subprocess, "run", return_value=response) as run:
            with self.assertRaises(RuntimeError):
                lifecycle.Docker()
        run.assert_called_once()
        self.assertEqual(run.call_args[0][0], ["docker", "context", "inspect"])

    def test_local_socket_is_pinned_and_remote_environment_is_removed(self):
        with patch.dict(os.environ, {"DOCKER_HOST": "unix:///local.sock", "DOCKER_TLS_VERIFY": "1"}, clear=True), \
                patch.object(Path, "is_absolute", return_value=True), \
                patch.object(Path, "exists", return_value=True), \
                patch.object(Path, "stat", return_value=SimpleNamespace(st_mode=stat.S_IFSOCK)), \
                patch.object(lifecycle.subprocess, "run", return_value=SimpleNamespace(returncode=0, stdout="ok", stderr="")) as run:
            docker = lifecycle.Docker()
            docker.call("image", "inspect", lifecycle.IMAGE)
        self.assertEqual(run.call_args[0][0][:3], ["docker", "--host", "unix:///local.sock"])
        self.assertNotIn("DOCKER_HOST", run.call_args[1]["env"])
        self.assertNotIn("DOCKER_TLS_VERIFY", run.call_args[1]["env"])


if __name__ == "__main__":
    unittest.main()
