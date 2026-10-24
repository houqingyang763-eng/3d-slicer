"""Asynchronous end-to-end acceptance executed inside the real Slicer process.

Prepare by importing this module. Only start() sends robot targets. All motion
goes through the panel's normal target controls and command handlers; screenshots
and frame snapshots are captured while Qt remains responsive.
"""
from pathlib import Path
import datetime
import json
import math
import time
import traceback

import numpy as np
import qt
import slicer
import vtk

PROJECT = Path(__file__).resolve().parents[1]


def copy_json(value):
    return json.loads(json.dumps(value))


def atomic_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def pose_error(actual, target):
    position_mm = float(np.linalg.norm(np.asarray(actual["position_m"]) - target["position_m"]) * 1000)
    qa = np.asarray(actual["orientation_xyzw"], dtype=float)
    qb = np.asarray(target["orientation_xyzw"], dtype=float)
    cosine = float(np.clip(abs(np.dot(qa / np.linalg.norm(qa), qb / np.linalg.norm(qb))), 0, 1))
    return position_mm, 2 * math.acos(cosine)


class SlicerAcceptance:
    def __init__(self):
        self.panel = slicer.ur12eSimulation
        self.folder = PROJECT / "output" / "slicer_acceptance" / datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.folder.mkdir(parents=True, exist_ok=True)
        self.stage = "prepared"
        self.stage_started = time.monotonic()
        self.samples = []
        self.report = {"evidence_type": "real_slicer_ros2_moveit_mock_hardware_loop",
                       "simulation_only": True, "actual_robot_connected": False,
                       "slicer_version": slicer.app.applicationVersion,
                       "output_directory": str(self.folder), "checks": {}}
        self.timer = qt.QTimer()
        self.timer.setInterval(200)
        self.timer.connect("timeout()", self.tick)
        self.write_status()

    def write_status(self):
        atomic_json(PROJECT / "output" / "slicer_acceptance_status.json",
                    {"stage": self.stage, "timestamp": time.time(), "output_directory": str(self.folder),
                     "report": self.report if self.stage in ("complete", "failed") else None})

    def transition(self, stage):
        self.stage, self.stage_started = stage, time.monotonic()
        self.write_status()

    def start(self, target_mm_deg=None):
        if self.stage != "prepared":
            raise RuntimeError("This acceptance instance has already started")
        self.panel.poll()
        state = self.panel.state
        if not state or state.get("source") != "ros2_mock_hardware" or not state.get("ready") or time.time() - state["timestamp"] > 3:
            raise RuntimeError("Fresh real ROS mock hardware state is required")
        if state.get("status") in ("planning", "executing"):
            raise RuntimeError("Another operation is active")
        self.report["initial_feedback"] = copy_json(state)
        self.report["started_at"] = time.time()
        if target_mm_deg is None:
            self.panel.copy_feedback_target()
            # The panel's copy helper adds +30 mm. Choose a small -25 mm Z
            # displacement for this recorded run, from the observed bent home.
            self.panel.position[2].setValue(float(self.panel.position[2].value) - 55.0)
            self.panel.angles[2].setValue(float(self.panel.angles[2].value) + 3.0)
        else:
            if len(target_mm_deg) != 6:
                raise ValueError("Target must be R,A,S millimetres and roll,pitch,yaw degrees")
            for spin, value in zip(self.panel.position + self.panel.angles, target_mm_deg):
                spin.setValue(float(value))
        self.report["target_from_slicer_controls"] = self.panel.current_target()
        self.report["target_control_values_mm_deg"] = [float(s.value) for s in self.panel.position + self.panel.angles]
        if self.panel.recording:
            self.panel.toggle_recording()
        self.panel.toggle_recording()
        # Keep this run's movie and manifest adjacent to the validation report.
        original = self.panel.recording
        self.panel.recording = self.folder / "frames"
        self.panel.recording.mkdir()
        if not any(original.iterdir()):
            original.rmdir()
        self.panel.apply_camera()
        self.panel.capture(self.folder / "01_target_set.png")
        self.transition("target_hold")
        self.timer.start()
        return {"stage": self.stage, "output_directory": str(self.folder)}

    def tick(self):
        try:
            state = self.panel.state
            if state:
                self.samples.append({"observed_at": time.time(), "observed_monotonic_s": time.monotonic(),
                                     "stage": self.stage, "state": copy_json(state)})
            if not state or state.get("source") != "ros2_mock_hardware" or not state.get("feedback_valid") or not state.get("ready"):
                raise RuntimeError("Real ROS joint/TF/MoveIt readiness was lost during Slicer acceptance")
            if time.time() - float(state.get("timestamp", 0)) > 2:
                raise RuntimeError("ROS file feedback became stale during Slicer acceptance")
            if state.get("joint_state_age_s") is None or state.get("tf_age_s") is None or max(state["joint_state_age_s"], state["tf_age_s"]) > 1:
                raise RuntimeError("Observed ROS joint/TF ages exceeded the 1 s freshness gate")
            elapsed = time.monotonic() - self.stage_started
            if self.stage == "target_hold" and elapsed > 2:
                self.report["plan_request"] = self.panel.send("plan")
                if self.report["plan_request"] is None:
                    raise RuntimeError("Panel refused the planning request")
                self.transition("planning")
            elif self.stage == "planning":
                if state and state.get("request_id") == self.report["plan_request"]["request_id"]:
                    if state.get("status") == "error":
                        raise RuntimeError("MoveIt planning failed: " + state.get("message", ""))
                    if state.get("status") == "planned" and len(state.get("plan", {}).get("tcp_path_m", [])) >= 2:
                        self.report["planned_feedback"] = copy_json(state)
                        self.panel.capture(self.folder / "02_moveit_plan.png")
                        self.transition("planned_hold")
                if elapsed > 45:
                    raise TimeoutError("Slicer did not observe the real plan and FK preview within 45 s")
            elif self.stage == "planned_hold" and elapsed > 3:
                if not self.panel.can_execute():
                    raise RuntimeError("Slicer did not enable execution of the plan for its current target")
                self.report["execute_request"] = self.panel.send("execute")
                if self.report["execute_request"] is None:
                    raise RuntimeError("Panel refused cached-plan execution")
                self.transition("executing")
            elif self.stage == "executing":
                if state and state.get("request_id") == self.report["execute_request"]["request_id"]:
                    if state.get("status") == "error":
                        raise RuntimeError("MoveIt execution failed: " + state.get("message", ""))
                    if state.get("status") == "succeeded":
                        self.transition("feedback_hold")
                if elapsed > 150:
                    raise TimeoutError("Slicer did not observe execution completion within 150 s")
            elif self.stage == "feedback_hold":
                if state.get("request_id") != self.report["execute_request"]["request_id"] or state.get("status") != "succeeded":
                    raise RuntimeError("Feedback during the hold no longer belongs to this successful execution")
                if elapsed > 4:
                    self.report["feedback_hold_duration_s"] = elapsed
                    self.finish()
        except Exception:
            self.report["error"] = traceback.format_exc()
            self.report["all_passed"] = False
            self.stop_recording()
            self.timer.stop()
            self.panel.capture(self.folder / "failure.png")
            atomic_json(self.folder / "results.json", self.report)
            atomic_json(self.folder / "observations.json", self.samples)
            self.transition("failed")

    def stop_recording(self):
        if self.panel.recording:
            self.report["recording_directory"] = str(self.panel.recording)
            self.report["recording_frame_count"] = len(self.panel.recorded_frames)
            self.panel.toggle_recording()

    def finish(self):
        self.timer.stop()
        state = copy_json(self.panel.state)
        target = self.panel.current_target()
        position_error_mm, orientation_error_rad = pose_error(state["tcp_pose"], target)
        orientation_error_deg = math.degrees(orientation_error_rad)
        actual = vtk.vtkMatrix4x4()
        self.panel.tcp_transform.GetMatrixTransformToParent(actual)
        actual_matrix = np.array([[actual.GetElement(i, j) for j in range(4)] for i in range(4)])
        from ur12e_panel import pose_matrix
        transform_difference = float(np.max(np.abs(actual_matrix - pose_matrix(state["tcp_pose"]))))
        execute_id = self.report["execute_request"]["request_id"]
        plan_id = self.report["plan_request"]["request_id"]
        before_execution = self.report["planned_feedback"]["joint_positions_rad"]
        execution_samples = [s for s in self.samples if s["stage"] in ("executing", "feedback_hold") and s["state"].get("request_id") == execute_id]
        joint_delta = max([float(np.max(np.abs(np.array(s["state"]["joint_positions_rad"]) - before_execution))) for s in execution_samples] or [0.0])
        hold_samples = [s for s in execution_samples if s["stage"] == "feedback_hold"]
        hold_errors = [pose_error(s["state"]["tcp_pose"], target) for s in hold_samples]
        hold_position_max_mm = max((error[0] for error in hold_errors), default=math.inf)
        hold_orientation_max_rad = max((error[1] for error in hold_errors), default=math.inf)
        hold_updates = len({s["state"]["timestamp"] for s in hold_samples})
        hold_joint_drift_rad = max((float(np.max(np.abs(np.asarray(s["state"]["joint_positions_rad"]) - hold_samples[0]["state"]["joint_positions_rad"]))) for s in hold_samples), default=math.inf)
        backend_target = state.get("target_pose") or {}
        target_matches = np.allclose(backend_target.get("position_m", [math.inf] * 3), target["position_m"], atol=1e-9, rtol=0) and abs(float(np.dot(backend_target.get("orientation_xyzw", [0] * 4), target["orientation_xyzw"]))) > 1-1e-9
        checks = {
            "genuine_ros2_mock_feedback": state.get("source") == "ros2_mock_hardware" and state.get("ready") and state.get("feedback_valid"),
            "slicer_target_roundtrip_unchanged": target == self.report["target_from_slicer_controls"],
            "moveit_plan_success": self.report["planned_feedback"]["plan"].get("moveit_error_code") == 1,
            "real_fk_preview_displayed": len(state["plan"].get("tcp_path_m", [])) >= 2 and bool(self.panel.plan_path.GetDisplayNode().GetVisibility()),
            "cached_plan_linked_to_execution": self.report["execute_request"].get("plan_request_id") == self.report["plan_request"]["request_id"],
            "final_backend_reply_matches_own_execution": state.get("request_id") == execute_id and state["timestamp"] >= self.report["execute_request"]["timestamp"],
            "final_backend_plan_matches_own_plan": state.get("plan", {}).get("request_id") == plan_id,
            "final_backend_target_matches_slicer_command": bool(target_matches),
            "moveit_execution_success": state["execution"].get("moveit_error_code") == 1 and state["execution"].get("action_status") == 4 and state.get("status") == "succeeded",
            "observed_joint_motion": joint_delta > 0.001,
            "position_error_at_most_2_mm": position_error_mm <= 2,
            "orientation_error_at_most_0_03_rad": math.radians(orientation_error_deg) <= 0.03,
            "feedback_hold_at_least_4_seconds": self.report.get("feedback_hold_duration_s", 0) >= 4,
            "feedback_hold_contains_live_updates": hold_updates >= 10,
            "feedback_hold_position_at_most_2_mm": hold_position_max_mm <= 2,
            "feedback_hold_orientation_at_most_0_03_rad": hold_orientation_max_rad <= 0.03,
            "slicer_tcp_transform_matches_feedback": transform_difference < 1e-7,
            "official_robot_meshes_displayed": len(self.panel.mesh_transforms) >= 7,
            "feedback_is_fresh": time.time() - state["timestamp"] < 2,
        }
        scene_file = self.folder / "UR12e_closed_loop.mrb"
        checks["scene_saved"] = bool(slicer.util.saveScene(str(scene_file)))
        self.panel.capture(self.folder / "03_execution_complete.png")
        self.stop_recording()
        checks["recorded_moving_demonstration"] = self.report.get("recording_frame_count", 0) >= 20
        self.report.update({"checks": checks, "all_passed": bool(all(checks.values())),
                            "final_feedback": state, "completed_at": time.time(),
                            "position_error_mm_independent": position_error_mm,
                            "orientation_error_deg_independent": orientation_error_deg,
                            "display_transform_max_abs_difference": transform_difference,
                            "max_observed_joint_delta_rad": joint_delta,
                            "execution_observed_sample_count": len(execution_samples),
                            "hold_observed_sample_count": len(hold_samples), "hold_distinct_feedback_updates": hold_updates,
                            "hold_position_error_max_mm_independent": hold_position_max_mm,
                            "hold_orientation_error_max_rad_independent": hold_orientation_max_rad,
                            "hold_joint_drift_max_rad": hold_joint_drift_rad,
                            "scene_file": str(scene_file), "observed_sample_count": len(self.samples),
                            "engineering_thresholds": {"position_mm": 2, "orientation_rad": 0.03},
                            "scope": "Mock hardware integration only; simulated coordinate relationship and simplified tool. No physical calibration or accuracy claim."})
        atomic_json(self.folder / "results.json", self.report)
        atomic_json(self.folder / "observations.json", self.samples)
        self.transition("complete" if self.report["all_passed"] else "failed")


def prepare():
    previous = getattr(slicer, "ur12eAcceptance", None)
    if previous and previous.timer.isActive():
        raise RuntimeError("An acceptance run is already active")
    slicer.ur12eAcceptance = SlicerAcceptance()
    return {"stage": "prepared", "output_directory": str(slicer.ur12eAcceptance.folder)}
