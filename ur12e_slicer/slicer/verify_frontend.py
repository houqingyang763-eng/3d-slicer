"""Run via scripts/slicer_client.py; isolated frontend checks, NOT ROS evidence.

Temporary test requests are written under output/frontend_verification only.
The application is restored to its real-feedback waiting screen afterwards.
"""
from pathlib import Path
import importlib
import json
import math
import re
import time
import uuid
import numpy as np
import vtk
import slicer
import ur12e_panel as frontend

if hasattr(slicer, "ur12eSimulation"):
    slicer.ur12eSimulation.stop()
    del slicer.ur12eSimulation
importlib.reload(frontend)
project = frontend.PROJECT
real_runtime, real_output = frontend.RUNTIME, frontend.OUTPUT
evidence = real_output / "frontend_verification"
evidence.mkdir(parents=True, exist_ok=True)
frontend.RUNTIME = evidence / ("isolated_runtime_" + uuid.uuid4().hex)
frontend.OUTPUT = evidence
panel = frontend.start()
panel.timer.stop()
checks = {}

try:
    panel.load_visual_meshes()
    checks["official_meshes_loaded"] = len(panel.mesh_transforms) == 7
    checks["no_feedback_means_no_robot_display"] = not any(node.GetDisplayNode().GetVisibility() for entries in panel.mesh_transforms.values() for node, _ in entries)
    # Quaternion construction and frame/unit conversion are tested in Slicer's
    # actual vtk/numpy environment, not by substituting a second implementation.
    for degrees in ([0, 0, 0], [180, 0, 0], [12, -34, 56], [-100, 60, 170]):
        rotation = frontend.rotation_rpy_degrees(*degrees)
        q = frontend.quaternion_from_rotation(rotation)
        assert np.allclose(frontend.rotation_from_quaternion(q), rotation, atol=1e-9)
    checks["orientation_roundtrip"] = True
    assert panel.send("plan_execute") is None
    checks["cannot_submit_without_ready_feedback"] = not (frontend.RUNTIME / "target.json").exists()

    # Explicit frontend fixture. This is analytic FK of the pinned UR12e model
    # used only to inspect visual placement and UI state handling.
    kinematics = {}
    section = None
    for line in (project / "assets/ur_description/config/ur12e/default_kinematics.yaml").read_text().splitlines():
        match = re.fullmatch(r"  (shoulder|upper_arm|forearm|wrist_1|wrist_2|wrist_3):", line)
        if match:
            section = match.group(1)
            kinematics[section] = {}
        match = re.fullmatch(r"    (x|y|z|roll|pitch|yaw):\s*(\S+)", line)
        if match and section:
            kinematics[section][match.group(1)] = float(match.group(2))
    q_joint = np.deg2rad([25, -100, 100, -90, -90, 0])
    transforms = {}
    base_link = np.eye(4)
    base_link[:3, :3] = frontend.rotation_rpy_degrees(0, 0, 180)
    transforms["base_link"] = base_link
    current = np.eye(4)  # base_link_inertia == the UR controller's base frame.
    for key, link, q in zip(["shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3"], frontend.LINK_ORDER[1:7], q_joint):
        values = kinematics[key]
        origin = np.eye(4)
        origin[:3, 3] = [values[axis] for axis in "xyz"]
        origin[:3, :3] = frontend.rotation_rpy_degrees(*np.rad2deg([values[name] for name in ("roll", "pitch", "yaw")]))
        joint = np.eye(4)
        joint[:3, :3] = frontend.rotation_rpy_degrees(0, 0, math.degrees(q))
        current = current @ origin @ joint
        transforms[link] = current.copy()
    flange = np.eye(4)
    flange[:3, :3] = frontend.rotation_rpy_degrees(0, -90, -90)
    tool = np.eye(4)
    tool[:3, :3] = frontend.rotation_rpy_degrees(90, 0, 90)
    transforms["tool0"] = current @ flange @ tool
    tip = np.eye(4)
    tip[:3, 3] = [0, 0, .12]
    transforms["needle_tip"] = transforms["tool0"] @ tip

    def pose(matrix):
        return {"frame_id": "base", "position_m": matrix[:3, 3].tolist(), "orientation_xyzw": frontend.quaternion_from_rotation(matrix[:3, :3])}

    target = panel.current_target()
    fixture = {"schema_version": 1, "source": "test_fixture", "timestamp": time.time(), "ready": True,
               "request_id": "frontend_test_fixture", "status": "idle", "message": "仅验证界面和几何，不代表 MoveIt 2 已运行。",
               "link_transforms": {link: pose(matrix) for link, matrix in transforms.items()},
               "tcp_pose": pose(transforms["needle_tip"]), "target_pose": target,
               "plan": {"status": "none", "point_count": 0}, "execution": {"status": "idle"},
               "tool": {"parent_frame": "tool0", "tip_offset_m": [0, 0, .12]}}
    panel.state = fixture
    panel.update_feedback(fixture)
    checks["fixture_is_visibly_labelled"] = "测试数据" in panel.status_label.text
    checks["actual_tool_transform_uses_fixture_feedback"] = np.allclose(frontend.pose_matrix(fixture["link_transforms"]["tool0"])[:3, 3], transforms["tool0"][:3, 3]*1000)
    panel.capture(evidence / "frontend_fixture_only.png")
    request = panel.send("plan")
    submitted = json.loads((frontend.RUNTIME / "target.json").read_text(encoding="utf-8"))
    checks["target_file_protocol"] = submitted == request and submitted["frame_id"] == "base" and len(submitted["orientation_xyzw"]) == 4
    fixture.update(status="planned", request_id=request["request_id"], target_pose=target, plan={"status":"succeeded", "request_id":request["request_id"], "point_count": 5})
    panel.update_feedback(fixture)
    checks["same_target_plan_can_execute"] = bool(panel.can_execute())
    panel.position[0].setValue(panel.position[0].value + 1)
    checks["changed_target_cannot_execute_old_plan"] = not panel.can_execute()
    panel.position[0].setValue(panel.position[0].value - 1)
    execution_request = panel.send("execute")
    checks["execute_references_real_plan_request_id"] = execution_request["plan_request_id"] == request["request_id"]
    panel.toggle_recording()
    panel.capture_recording_frame()
    recording_directory = panel.recording
    panel.toggle_recording()
    checks["frame_recording_writes_images_and_manifest"] = (recording_directory / "frames.json").exists() and (recording_directory / "frame_00000.png").exists()
    scene_directory = Path(panel.save())
    checks["scene_and_feedback_export"] = (scene_directory / "UR12e_simulation.mrb").exists() and (scene_directory / "results.json").exists()
    fixture["timestamp"] = time.time()-10
    panel.show_errors(fixture)
    checks["stale_feedback_is_marked"] = "已过期" in panel.error_label.text
    assert all(checks.values()), checks
finally:
    panel.stop()
    del slicer.ur12eSimulation
    frontend.RUNTIME, frontend.OUTPUT = real_runtime, real_output
    restored = frontend.start()
    restored.load_visual_meshes()
    restored.capture(real_output / "frontend_waiting_for_ros.png")

report = {"evidence_type": "frontend_only_with_explicit_fixture", "ros2_moveit_executed": False,
          "actual_robot_connected": False, "checks": checks, "all_passed": all(checks.values()),
          "fixture_screenshot": str(evidence / "frontend_fixture_only.png"),
          "waiting_screenshot": str(real_output / "frontend_waiting_for_ros.png")}
frontend.atomic_json(evidence / "results.json", report)
_result = report
