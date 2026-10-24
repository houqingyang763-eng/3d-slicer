#!/usr/bin/env python3
"""Run image/optical calibration -> genuine MoveIt -> mock feedback.

Run under WSL to share the ROS clock. Independent virtual geometry supplies
optical poses; it is not an independent physical sensor or clinical ground
truth. Never connect this runner to real hardware.
"""
import argparse
import copy
import csv
import hashlib
import html
import json
from pathlib import Path
import sys
import time
import traceback
import uuid

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "validation"))
from navigation_benchmark.core import (ObservationRejected, estimated_robot_goal,
    invert, matrix_from_pose, optical_tip_in_image, pose_errors, pose_from_matrix,
    rigid_registration, transform_points)
from navigation_benchmark.gateway import NavigationGate
from validate_ros_runtime import Validation, atomic_json


class NavigationValidation(Validation):
    def read_state(self):
        # DrvFS can briefly hide a file during atomic replacement. Retry only
        # these storage errors, never stale/invalid data or failed assertions.
        deadline = time.monotonic() + 0.25
        while True:
            try:
                return super().read_state()
            except (FileNotFoundError, PermissionError):
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def rotation_xyz(rx, ry, rz):
    x, y, z = np.deg2rad([rx, ry, rz])
    cx, cy, cz, sx, sy, sz = np.cos(x), np.cos(y), np.cos(z), np.sin(x), np.sin(y), np.sin(z)
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array(
        [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]) @ np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])


def transform(position, angles):
    value = np.eye(4)
    value[:3, :3] = rotation_xyz(*angles)
    value[:3, 3] = position
    return value


def build_world(initial_state):
    tcp = initial_state["tcp_pose"]
    target_B = matrix_from_pose(np.array(tcp["position_m"]) * 1000, tcp["orientation_xyzw"])
    target_B[2, 3] -= 25.0
    # Place the public MRI near the observed robot, but retain its own RAS
    # coordinate system. These arbitrary fiducials are not anatomical labels.
    T_B_I = transform([0, 0, 0], [10, -15, 30])
    image_target = np.array([0, 0, 30.0])
    T_B_I[:3, 3] = target_B[:3, 3] - T_B_I[:3, :3] @ image_target
    target_I = invert(T_B_I) @ target_B
    T_B_C = transform([-400, 300, 950], [-15, 10, -35])
    T_H_I = transform([35, -20, 15], [8, -12, 17])
    T_C_H = invert(T_B_C) @ T_B_I @ invert(T_H_I)
    T_M_Tip = transform([12, -8, 120], [4, -7, 11])
    fiducials = [[-55, -50, 20], [55, -50, 20], [-50, 50, 55],
                 [50, 50, 55], [-40, 0, -60], [40, 0, -60]]
    targets = [[0, 0, 30], [10, 0, 30], [-10, 0, 30], [0, 10, 30], [0, -10, 30]]
    truth = {"T_B_I": T_B_I, "T_B_C": T_B_C, "T_H_I": T_H_I,
             "T_C_H": T_C_H, "T_M_Tip": T_M_Tip, "target_I": target_I,
             "fiducials_image_mm": fiducials, "targets_image_mm": targets,
             "truth_source": "synthetic rigid world anchored to initial mock FK; not physical metrology"}
    return truth


def observe(truth, state):
    tcp = state["tcp_pose"]
    actual = matrix_from_pose(np.array(tcp["position_m"]) * 1000, tcp["orientation_xyzw"])
    # A separate immutable virtual world, not the estimated calibration,
    # generates marker observations from actual mock feedback.
    tool = invert(truth["T_B_C"]) @ actual @ invert(truth["T_M_Tip"])
    now = time.time()
    # Keep the original FK sampling time. The virtual head is static at the
    # same historical instant, so both simulated objects share that instant.
    stamp = state["timestamp"] - max(state["joint_state_age_s"], state["tf_age_s"])
    return {"now_s": now,
            "head": {"matrix": np.array(truth["T_C_H"], copy=True), "timestamp_s": stamp,
                     "valid": True, "clock_domain": "wsl_wall_simulation"},
            "tool": {"matrix": tool, "timestamp_s": stamp, "valid": True,
                     "clock_domain": "wsl_wall_simulation"}}


def read_retry(gate):
    # Retry only short shared-file visibility failures, retaining original
    # freshness checks. Invalid/stale feedback is not retried away.
    deadline = time.monotonic() + 0.25
    while True:
        try:
            return gate.read_state()
        except (FileNotFoundError, PermissionError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.01)


def fresh_observation(truth, bridge):
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        state = read_retry(bridge)
        observation = observe(truth, state)
        age = time.time() - observation["tool"]["timestamp_s"]
        if 0 <= age < 0.065:
            return state, observation
        time.sleep(0.01)
    raise AssertionError("No sufficiently fresh FK sample for the assumed 100 ms optical gate")


def run(args):
    adapter_args = argparse.Namespace(runtime=PROJECT / "runtime",
        output=args.output, position_tolerance=0.002, orientation_tolerance=0.03,
        max_feedback_age=1.0, ready_timeout=30.0, step_timeout=90.0)
    bridge = NavigationValidation(adapter_args)
    output = bridge.output
    result = {"schema_version": 1, "mode": "simulation", "passed": False,
              "run_id": bridge.run_id, "started_at": time.time(), "cases": [],
              "limitations": ["synthetic optical stream, not a physical tracker",
                "mock robot feedback comes from UR kinematics, not independent physical measurement",
                "public MRI with synthetic fiducials and targets; no head collision model",
                "application gate can be bypassed by direct generic bridge commands",
                "no motion-during-optical-loss or stopping-latency validation",
                "not an ASTM conformity test or clinical accuracy result"]}
    context_gate = NavigationGate(clock_domain="wsl_wall_simulation")
    lock = PROJECT / "runtime" / "navigation_validation.lock"
    locked = False
    last_state = None
    try:
        with lock.open("x", encoding="utf-8") as stream:
            json.dump({"run_id": bridge.run_id, "started_at": time.time()}, stream)
        locked = True
        initial = bridge.wait_ready()
        world_path = args.truth or (args.output / "reference_world.json")
        if world_path.exists():
            truth = json.loads(world_path.read_text(encoding="utf-8"))
            for key in ("T_B_I", "T_B_C", "T_H_I", "T_C_H", "T_M_Tip", "target_I"):
                truth[key] = np.asarray(truth[key], dtype=float)
        else:
            truth = build_world(initial)
            atomic_json(world_path, json_safe(truth))
        result["reference_world_path"] = str(world_path)
        calibration = {key: np.array(truth[key], copy=True) for key in ("T_B_C", "T_H_I", "T_M_Tip")}
        fiducials_I = np.asarray(truth["fiducials_image_mm"], dtype=float)
        measured_fiducials_H = transform_points(truth["T_H_I"], fiducials_I)
        calibration["T_H_I"], fre = rigid_registration(fiducials_I, measured_fiducials_H)
        heldout_I = np.asarray(truth["targets_image_mm"], dtype=float)
        assert not any(np.allclose(a, b) for a in fiducials_I for b in heldout_I), "Fitted and held-out points overlap"
        fitted_targets = transform_points(calibration["T_H_I"], heldout_I)
        true_targets = transform_points(truth["T_H_I"], heldout_I)
        result["registration"] = {"method": "rigid_Kabsch_no_scale", "fiducial_count": len(fiducials_I),
            "heldout_count": len(heldout_I), "fre_mm": fre,
            "heldout_tre_rms_mm": float(np.sqrt(np.mean(np.sum((fitted_targets-true_targets)**2, axis=1)))),
            "measured_fiducials_H_mm": measured_fiducials_H.tolist(),
            "source": "synthetic noiseless point observations, not physical fiducial measurements"}
        frozen = json.dumps(json_safe(truth), sort_keys=True, allow_nan=False).encode("utf-8")
        result["truth_sha256"] = hashlib.sha256(frozen).hexdigest()
        atomic_json(output / "truth.json", json_safe(truth))
        atomic_json(output / "initial_state.json", initial)
        all_samples = []
        for index, label in enumerate(("ideal_calibration", "camera_extrinsic_3mm", "calibration_restored")):
            current_cal = copy.deepcopy(calibration)
            if label == "camera_extrinsic_3mm":
                current_cal["T_B_C"][0, 3] += 3.0
            target_I = np.array(truth["target_I"], copy=True)
            # Three distinct nearby target poses, all derived from an image
            # goal through optical head observation and calibration.
            target_I[0, 3] += (0, 6, -6)[index]
            fresh, observation = fresh_observation(truth, bridge)
            planned_context = context_gate.plan_context(current_cal, observation, target_I)
            commanded = np.array(planned_context["goal_base_mm"])
            true_goal = truth["T_B_I"] @ target_I
            xyz, quat = pose_from_matrix(commanded)
            pose = {"position_m": (np.array(xyz)/1000).tolist(), "orientation_xyzw": quat}
            request = bridge.submit("plan", pose, label + "_plan")
            terminal, statuses, plan_samples = bridge.wait_terminal(request)
            assert terminal["status"] == "planned", terminal.get("message")
            preview_state = bridge.wait_preview(request["request_id"])
            # Re-observe after planning, rather than authorizing with the
            # optical frame used several seconds before the planning call.
            unused_state, observation = fresh_observation(truth, bridge)
            context_gate.authorize_execute(planned_context, current_cal, observation, target_I)
            execute = bridge.submit("execute", pose, label + "_execute", request["request_id"])
            terminal, exec_statuses, exec_samples = bridge.wait_terminal(execute)
            assert terminal["status"] == "succeeded", terminal.get("message")
            terminal, observed = fresh_observation(truth, bridge)
            assert terminal["request_id"] == execute["request_id"], "Final state replaced by another sender"
            assert terminal["plan"]["request_id"] == request["request_id"], "Final cached plan changed"
            assert terminal["execution"]["action_status"] == 4, "Execute action did not succeed"
            actual = matrix_from_pose(np.array(terminal["tcp_pose"]["position_m"])*1000,
                                      terminal["tcp_pose"]["orientation_xyzw"])
            optical_I = optical_tip_in_image(current_cal, observed, now_s=time.time())
            measured = pose_errors(optical_I, target_I)
            commanded_error = pose_errors(actual, commanded)
            assert commanded_error["position_error_mm"] <= 2.0, commanded_error
            assert commanded_error["orientation_error_deg"] <= np.degrees(0.03), commanded_error
            world_error = pose_errors(actual, true_goal)
            joint_change = float(np.max(np.abs(np.array(terminal["joint_positions_rad"])-np.array(fresh["joint_positions_rad"]))))
            assert joint_change > 1e-6, "No joint movement was observed for a distinct target"
            assert abs(world_error["position_error_mm"]-measured["position_error_mm"]) < 1e-7
            predicted_bias = pose_errors(commanded, true_goal)["position_error_mm"]
            expected_bias = 3.0 if index == 1 else 0.0
            assert abs(predicted_bias - expected_bias) < 1e-7
            # Exact vector decomposition separates known calibration bias
            # from the genuine planner's residual, without subtracting RMSs.
            vector = actual[:3, 3]-true_goal[:3, 3]
            decomposition = (actual[:3, 3]-commanded[:3, 3])+(commanded[:3, 3]-true_goal[:3, 3])
            assert np.linalg.norm(vector-decomposition) < 1e-8
            case = {"id": label, "passed": True, "known_command_bias_mm": predicted_bias,
                "max_joint_change_rad": joint_change,
                "true_target_error": world_error, "optical_estimated_error": measured,
                "robot_command_residual": commanded_error, "target_I": target_I.tolist(),
                "goal_base_mm": true_goal.tolist(), "commanded_goal_base_mm": commanded.tolist(),
                "actual_tip_base_mm": actual.tolist(),
                "optical_tip_base_mm": (truth["T_B_I"] @ optical_I).tolist(),
                "calibration": json_safe(current_cal), "observations": json_safe(observed),
                "plan_request": request, "execute_request": execute,
                "plan_point_count": terminal["plan"].get("point_count"),
                "planned_tip_path_m": preview_state["plan"].get("tcp_path_m", []),
                "observed_statuses": statuses + exec_statuses,
                "robot_feedback_snapshot": terminal}
            result["cases"].append(case)
            atomic_json(output / (label + ".json"), case)
            for state in exec_samples + [terminal]:
                if state.get("tcp_pose"):
                    all_samples.append({"timestamp_s": state["timestamp"], "case": label,
                                        "tcp_pose": state["tcp_pose"]})
            last_state = terminal
            print("{}: command {:.3f} mm; image target {:.3f} mm; imposed bias {:.3f} mm".format(
                  label, commanded_error["position_error_mm"], world_error["position_error_mm"], predicted_bias), flush=True)

        # These cases must fail before writing an execute request. They test
        # the application authorization boundary; no stop action is simulated.
        unused_state, current_observation = fresh_observation(truth, bridge)
        target_I = np.array(truth["target_I"], copy=True)
        context = context_gate.plan_context(calibration, current_observation, target_I)
        invalid_cases = []
        for label, expected in (("tool_occluded", "tracking_invalid"),
                                ("old_pair", "stale_frame"),
                                ("whole_packet_replay", "stale_frame"),
                                ("head_tool_skew", "frame_skew"),
                                ("head_moved_after_plan", "head_moved"),
                                ("calibration_replaced", "calibration_changed")):
            before = (PROJECT / "runtime" / "target.json").read_bytes()
            before_state = read_retry(bridge)
            unused_state, obs = fresh_observation(truth, bridge)
            changed_cal = copy.deepcopy(calibration)
            if label == "tool_occluded":
                obs["tool"]["valid"] = False
            elif label == "old_pair":
                obs["head"]["timestamp_s"] -= 0.2
                obs["tool"]["timestamp_s"] -= 0.2
            elif label == "whole_packet_replay":
                obs["now_s"] -= 0.2
                obs["head"]["timestamp_s"] -= 0.2
                obs["tool"]["timestamp_s"] -= 0.2
            elif label == "head_tool_skew":
                obs["head"]["timestamp_s"] = time.time() - 0.005
                obs["tool"]["timestamp_s"] = time.time() - 0.050
            elif label == "head_moved_after_plan":
                obs["head"]["matrix"][0, 3] += 5.0
            elif label == "calibration_replaced":
                changed_cal["T_B_C"][1, 3] += 1.0
            try:
                context_gate.authorize_execute(context, changed_cal, obs, target_I)
            except ObservationRejected as exc:
                code = exc.code
            else:
                raise AssertionError("Invalid case authorized: " + label)
            assert code == expected, (label, code)
            after = (PROJECT / "runtime" / "target.json").read_bytes()
            assert before == after, "Rejected case wrote a robot command"
            after_state = read_retry(bridge)
            drift = float(np.max(np.abs(np.array(after_state["joint_positions_rad"])-np.array(before_state["joint_positions_rad"]))))
            assert drift < 1e-6, "Robot moved during a rejected case"
            invalid_cases.append({"id": label, "passed": True, "rejection": code,
                                  "motion_command_written": False, "max_joint_change_rad": drift,
                                  "position_error_mm": None})
        result["rejection_cases"] = invalid_cases
        assert hashlib.sha256(json.dumps(json_safe(truth), sort_keys=True, allow_nan=False).encode("utf-8")).hexdigest() == result["truth_sha256"]
        result["samples"] = all_samples
        final = result["cases"][-1]
        result.update({"head": {"volume_path": "data/MR-head.nrrd",
                "T_base_image_mm": truth["T_B_I"].tolist(),
                "T_base_camera_mm": truth["T_B_C"].tolist(),
                "T_base_reference_mm": (truth["T_B_C"] @ truth["T_C_H"]).tolist(),
                "fiducials_image_mm": truth["fiducials_image_mm"],
                "targets_image_mm": truth["targets_image_mm"],
                "fiducials_observed_base_mm": transform_points(truth["T_B_I"], truth["fiducials_image_mm"]).tolist()},
            "goal_base_mm": final["goal_base_mm"], "actual_tip_base_mm": final["actual_tip_base_mm"],
            "optical_tip_base_mm": final["optical_tip_base_mm"],
            "robot_feedback_snapshot": last_state,
            "summary": {"executed_cases": 3, "rejected_without_command": len(invalid_cases),
                "final_true_target_mm": final["true_target_error"]["position_error_mm"],
                "final_axis_error_deg": final["true_target_error"]["axis_error_deg"],
                "known_bias_case_mm": 3.0, "optical_source": "SIMULATED"}, "passed": True})
    except Exception:
        result["error"] = traceback.format_exc()
        print(result["error"], flush=True)
    finally:
        bridge.trace.close()
        if locked:
            lock.unlink(missing_ok=True)
        result["finished_at"] = time.time()
        atomic_json(output / "run.json", json_safe(result))
        if result["passed"]:
            atomic_json(args.output / "latest_result.json", json_safe(result))
        rows = []
        for case in result["cases"]:
            rows.append({"case": case["id"], "passed": case["passed"],
                "target_error_mm": case["true_target_error"]["position_error_mm"],
                "axis_error_deg": case["true_target_error"]["axis_error_deg"],
                "command_residual_mm": case["robot_command_residual"]["position_error_mm"],
                "known_command_bias_mm": case["known_command_bias_mm"]})
        with (output / "cases.csv").open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=["case", "passed", "target_error_mm", "axis_error_deg", "command_residual_mm", "known_command_bias_mm"])
            writer.writeheader()
            writer.writerows(rows)
        body = "".join("<tr>"+"".join("<td>"+html.escape(str(value))+"</td>" for value in row.values())+"</tr>" for row in rows)
        report = '<!doctype html><meta charset="utf-8"><title>头模导航仿真报告</title><style>body{font:16px system-ui;max-width:1000px;margin:40px auto;line-height:1.7}td,th{padding:10px;border:1px solid #ccc}table{border-collapse:collapse}</style><h1>头模导航仿真：真实 MoveIt 与模拟光学</h1><p>这是软件验证，非实物精度、非 ASTM 合规测试。</p><p>列：场景／通过／真实目标误差 mm／轴向误差 °／机器人命令残差 mm／已知外参偏差 mm</p><table>'+body+'</table><h2>证明范围</h2><ul>'+"".join("<li>"+html.escape(item)+"</li>" for item in result["limitations"])+"</ul><p>完整输入、真值、请求、状态与拒绝记录见同目录 run.json。</p>"
        (output / "report.html").write_text(report, encoding="utf-8")
    print(json.dumps({"passed": result["passed"], "output": str(output)}, ensure_ascii=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=PROJECT / "output" / "navigation_benchmark")
    parser.add_argument("--truth", type=Path, default=PROJECT / "navigation_benchmark" / "fixtures" / "head_world_v1.json",
                        help="Reuse a saved virtual world instead of anchoring a new one")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sys.exit(run(args))
