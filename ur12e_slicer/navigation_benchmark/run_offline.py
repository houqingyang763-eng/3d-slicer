"""Deterministic simulated navigation evaluation, with separately saved truth.

Usage: python run_offline.py --output ABSOLUTE_PATH [--fixture fixture.json]
"""

import argparse
import copy
import csv
import hashlib
import html
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    from .core import (ObservationRejected, estimated_robot_goal, invert,
                       matrix_from_pose, optical_tip_in_image, pose_errors,
                       rigid_registration, transform_points, validate_observations,
                       validate_execution_snapshot)
except ImportError:
    from core import (ObservationRejected, estimated_robot_goal, invert,
                      matrix_from_pose, optical_tip_in_image, pose_errors,
                      rigid_registration, transform_points, validate_observations,
                      validate_execution_snapshot)


SCENARIOS = [
    ("ideal", "无扰动"),
    ("registration_single_point", "一个配准点偏移 4 mm"),
    ("registration_shared_bias", "全部配准点同向偏移 2.5 mm"),
    ("camera_extrinsic_3mm", "相机至底座外参平移偏差 3 mm"),
    ("tool_calibration_bias", "工具标定偏差 2.5 mm / 3°"),
    ("head_pose_updated", "头模移动后重新生成目标"),
    ("head_moved_after_plan", "规划后头模移动，拒绝旧命令"),
    ("tool_occluded", "工具标记遮挡"),
    ("head_occluded", "头部标记遮挡"),
    ("stale_pair", "两路观测同样过期"),
    ("cross_object_skew", "头部与工具观测时间不齐"),
    ("known_dynamic_delay", "头模 20 mm/s，观测延迟 80 ms"),
]


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(v) for v in value]
    return value


def canonical(value):
    return json.dumps(jsonable(value), sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def sha256(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def save_json(path, value):
    path.write_text(json.dumps(jsonable(value), ensure_ascii=False, indent=2,
                               allow_nan=False) + "\n", encoding="utf-8")


def pose_xyz_axis_angle(position, axis, degrees):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    angle = math.radians(degrees) / 2
    return matrix_from_pose(position, list(axis * math.sin(angle)) + [math.cos(angle)])


def observation(matrix, timestamp_s=20.0, valid=True):
    return {"matrix": jsonable(matrix), "timestamp_s": timestamp_s,
            "valid": valid, "clock_domain": "simulation_seconds"}


def make_fixture(seed=20261023):
    """Nonidentity rotations/translations; calibration and targets are disjoint."""
    random = np.random.RandomState(seed)
    fiducials_I = np.asarray([
        [-65, -55, -20], [61, -52, -22], [-60, 49, -18], [62, 54, -15],
        [-48, -35, 72], [52, -29, 76], [-45, 41, 65], [47, 44, 70],
    ], dtype=float) + random.uniform(-2, 2, (8, 3))
    heldout_points_I = np.asarray([
        [-22, -17, 26], [25, -13, 29], [-18, 21, 39],
        [24, 19, 45], [1, -1, 58], [4, 6, 8],
    ], dtype=float) + random.uniform(-1, 1, (6, 3))
    truth = {
        "T_B_C": pose_xyz_axis_angle([250, -160, 380], [1, 2, -1], 31),
        "T_C_H": pose_xyz_axis_angle([-80, 60, 510], [2, -1, 3], -23),
        "T_H_I": pose_xyz_axis_angle([34, -26, 18], [-1, 3, 2], 17),
        "T_M_Tip": pose_xyz_axis_angle([12, -8, 120], [1, 1, 2], 12),
        "fiducials_I": fiducials_I,
        "targets_I": [{"id": "heldout_{:02d}".format(i+1),
                       "matrix": pose_xyz_axis_angle(point, [1+i, 2, -1], 11+i*3)}
                      for i, point in enumerate(heldout_points_I)],
    }
    truth["fiducials_H"] = transform_points(truth["T_H_I"], fiducials_I)
    fitted, fre = rigid_registration(truth["fiducials_I"], truth["fiducials_H"])
    calibration = {"T_B_C": truth["T_B_C"].copy(), "T_H_I": fitted,
                   "T_M_Tip": truth["T_M_Tip"].copy()}
    # Initial tool pose is arbitrary and separate from the goal.
    initial_tool_C = pose_xyz_axis_angle([110, -70, 590], [1, 2, 1], 37)
    return jsonable({
        "schema": "ur12e_navigation_fixture_v1", "seed": seed,
        "units": "mm", "transform_convention": "T_A_B maps B to A",
        "truth": truth, "calibration": calibration,
        "fiducial_ids": ["fit_{:02d}".format(i+1) for i in range(len(fiducials_I))],
        "heldout_ids": [item["id"] for item in truth["targets_I"]],
        "registration_fre_mm": fre,
        "observations": {"now_s": 20.0,
                         "head": observation(truth["T_C_H"]),
                         "tool": observation(initial_tool_C)},
        "policy_assumptions": {"max_age_s": 0.100, "max_skew_s": 0.030,
                               "head_motion_translation_mm": 0.5,
                               "head_motion_rotation_deg": 0.5},
        "evidence_boundary": "Synthetic geometry and perfect marker-pose execution; no measured device performance, no ASTM conformity claim.",
    })


def check_fixture(fixture):
    if fixture.get("schema") != "ur12e_navigation_fixture_v1":
        raise ValueError("Unsupported fixture schema")
    ids = fixture["heldout_ids"]
    if set(ids) & set(fixture["fiducial_ids"]):
        raise ValueError("Fitting and held-out target IDs overlap")
    if ids != [target["id"] for target in fixture["truth"]["targets_I"]]:
        raise ValueError("Held-out target IDs do not match target records")
    # Coordinates must also differ: renaming fitting points cannot make a TRE set.
    fit = np.asarray(fixture["truth"]["fiducials_I"], dtype=float)
    for target in fixture["truth"]["targets_I"]:
        point = np.asarray(target["matrix"], dtype=float)[:3, 3]
        if np.any(np.linalg.norm(fit-point, axis=1) < 1e-8):
            raise ValueError("A held-out target reuses a fitted fiducial coordinate")


def evaluate_case(fixture, scenario, target_record):
    """No command is executed when tracking or snapshot authorization is invalid."""
    truth = copy.deepcopy(fixture["truth"])
    calibration = copy.deepcopy(fixture["calibration"])
    observations = copy.deepcopy(fixture["observations"])
    target_I = np.asarray(target_record["matrix"], dtype=float)
    current_head_C = np.asarray(truth["T_C_H"], dtype=float)
    planned_head_C = current_head_C.copy()
    fre = float(fixture["registration_fre_mm"])
    analytic_error_mm = None
    if scenario in ("registration_single_point", "registration_shared_bias"):
        measured = np.asarray(truth["fiducials_H"], dtype=float).copy()
        if scenario == "registration_single_point":
            measured[0, 2] += 4.0
        else:
            measured += [0, 2.5, 0]
            analytic_error_mm = 2.5
        calibration["T_H_I"], fre = rigid_registration(truth["fiducials_I"], measured)
    elif scenario == "camera_extrinsic_3mm":
        calibration["T_B_C"] = np.asarray(calibration["T_B_C"], dtype=float).copy()
        calibration["T_B_C"][:3, 3] += [3, 0, 0]
        analytic_error_mm = 3.0
    elif scenario == "tool_calibration_bias":
        offset = pose_xyz_axis_angle([0, 0, 2.5], [1, 0, 0], 3)
        calibration["T_M_Tip"] = np.asarray(calibration["T_M_Tip"]).dot(offset)
        analytic_error_mm = 2.5
    elif scenario in ("head_pose_updated", "head_moved_after_plan"):
        current_head_C = pose_xyz_axis_angle([7, -4, 3], [0, 1, 0], 6).dot(current_head_C)
        observations["head"] = observation(current_head_C)
    elif scenario == "tool_occluded":
        observations["tool"]["valid"] = False
    elif scenario == "head_occluded":
        observations["head"]["valid"] = False
    elif scenario == "stale_pair":
        observations["head"]["timestamp_s"] -= 0.250
        observations["tool"]["timestamp_s"] -= 0.250
    elif scenario == "cross_object_skew":
        observations["tool"]["timestamp_s"] -= 0.050
    elif scenario == "known_dynamic_delay":
        # Head moves in camera +X at exactly 20 mm/s; command uses an 80ms-old
        # observation pair. Delay is within the assumed 100ms validity gate.
        lag_s, velocity_mm_s = 0.080, 20.0
        current_head_C = current_head_C.copy()
        current_head_C[:3, 3] += [lag_s * velocity_mm_s, 0, 0]
        observations["head"]["timestamp_s"] -= lag_s
        observations["tool"]["timestamp_s"] -= lag_s
        analytic_error_mm = lag_s * velocity_mm_s
    row = {"scenario": scenario, "target_id": target_record["id"],
           "status": "rejected", "rejection_code": None,
           "registration_fre_mm": fre,
           "true_position_error_mm": None, "display_position_error_mm": None,
           "true_axis_error_deg": None, "display_axis_error_deg": None,
           "true_orientation_error_deg": None, "display_orientation_error_deg": None,
           "analytic_position_error_mm": analytic_error_mm,
           "actual_tip_I": None, "display_tip_I": None, "goal_tip_B": None,
           "current_head_C": jsonable(current_head_C)}
    try:
        policy = fixture["policy_assumptions"]
        gate = {"max_age_s": policy["max_age_s"], "max_skew_s": policy["max_skew_s"]}
        validate_observations(observations, **gate)
        if scenario == "head_moved_after_plan":
            validate_execution_snapshot(planned_head_C, observations,
                max_translation_mm=policy["head_motion_translation_mm"],
                max_rotation_deg=policy["head_motion_rotation_deg"], **gate)
        goal_B = estimated_robot_goal(calibration, observations, target_I, **gate)
        # Perfect execution is explicitly assumed ONLY for marker M. Tool tip
        # truth remains independently calibrated and is never used for command.
        commanded_marker_B = goal_B.dot(invert(calibration["T_M_Tip"]))
        actual_tip_B = commanded_marker_B.dot(np.asarray(truth["T_M_Tip"], dtype=float))
        actual_tip_I = (invert(truth["T_H_I"]).dot(invert(current_head_C))
                        .dot(invert(truth["T_B_C"])).dot(actual_tip_B))
        observed_marker_C = invert(truth["T_B_C"]).dot(commanded_marker_B)
        # The command is held stationary; the delayed tool pose equals its
        # current pose in this deliberately isolated constant-head-velocity test.
        observations["tool"]["matrix"] = jsonable(observed_marker_C)
        display_tip_I = optical_tip_in_image(calibration, observations, **gate)
        true_errors = pose_errors(actual_tip_I, target_I)
        display_errors = pose_errors(display_tip_I, target_I)
        row.update({"status": "valid", "actual_tip_I": jsonable(actual_tip_I),
                    "display_tip_I": jsonable(display_tip_I), "goal_tip_B": jsonable(goal_B)})
        for name in ("position_error_mm", "axis_error_deg", "orientation_error_deg"):
            row["true_"+name] = true_errors[name]
            row["display_"+name] = display_errors[name]
    except ObservationRejected as exc:
        row["rejection_code"] = exc.code
    return row


def summarize(rows):
    summaries = []
    for scenario, title in SCENARIOS:
        group = [row for row in rows if row["scenario"] == scenario]
        valid = [row for row in group if row["status"] == "valid"]
        summary = {"scenario": scenario, "title": title, "count": len(group),
                   "valid": len(valid), "rejected": len(group)-len(valid),
                   "rejection_codes": sorted(set(row["rejection_code"] for row in group
                                                  if row["rejection_code"] is not None))}
        for column in ("true_position_error_mm", "display_position_error_mm",
                       "true_axis_error_deg", "true_orientation_error_deg", "registration_fre_mm"):
            values = [row[column] for row in valid]
            summary[column+"_mean"] = float(np.mean(values)) if values else None
        summaries.append(summary)
    return summaries


def oracle_checks(rows):
    """Analytic values use norm invariance and speed x delay, not chain replay."""
    checks = []
    for scenario in ("ideal", "head_pose_updated"):
        group = [row for row in rows if row["scenario"] == scenario]
        checks.append({"name": scenario+"_zero_position", "passed": all(
            row["status"] == "valid" and row["true_position_error_mm"] < 1e-7 for row in group)})
    for scenario, expected in (("registration_shared_bias", 2.5),
                               ("camera_extrinsic_3mm", 3.0),
                               ("tool_calibration_bias", 2.5),
                               ("known_dynamic_delay", 1.6)):
        group = [row for row in rows if row["scenario"] == scenario]
        checks.append({"name": scenario+"_analytic_translation", "expected_mm": expected,
                       "passed": all(row["status"] == "valid" and
                       abs(row["true_position_error_mm"]-expected) < 1e-7 for row in group)})
    expected_rejections = {"head_moved_after_plan": "head_pose_changed",
        "tool_occluded": "tracking_invalid", "head_occluded": "tracking_invalid",
        "stale_pair": "stale_frame", "cross_object_skew": "frame_skew"}
    for scenario, code in expected_rejections.items():
        checks.append({"name": scenario+"_rejects", "passed": all(
            row["status"] == "rejected" and row["rejection_code"] == code and
            row["true_position_error_mm"] is None and row["goal_tip_B"] is None
            for row in rows if row["scenario"] == scenario)})
    # Shared registration and tool errors can look perfect in the estimated
    # display, while truth disagrees. Base-camera error remains observable.
    for scenario in ("registration_shared_bias", "tool_calibration_bias", "known_dynamic_delay"):
        checks.append({"name": scenario+"_display_is_not_truth", "passed": all(
            row["display_position_error_mm"] < 1e-7 and row["true_position_error_mm"] > 1
            for row in rows if row["scenario"] == scenario)})
    return checks


def write_report(path, run):
    def number(value):
        return "—" if value is None else "{:.4f}".format(value)
    body_rows = []
    for summary in run["summary"]:
        body_rows.append("<tr><td>{}</td><td>{}/{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>".format(
            html.escape(summary["title"]), summary["valid"], summary["count"],
            number(summary["registration_fre_mm_mean"]),
            number(summary["true_position_error_mm_mean"]),
            number(summary["display_position_error_mm_mean"]),
            number(summary["true_axis_error_deg_mean"]),
            html.escape(", ".join(summary["rejection_codes"])) or "有效"))
    document = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>离线导航几何评价（模拟）</title><style>body{font:16px/1.65 system-ui,sans-serif;margin:32px auto;max-width:1250px;padding:0 24px;color:#152536}h1{font-size:27px}.note{background:#fff3cf;padding:15px;border-left:5px solid #d78c13}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #d4dee8;padding:9px;text-align:left}th{background:#e9f1f7}code{background:#eef2f6;padding:2px 5px}small{color:#536579}</style>
<h1>离线导航几何评价：全部结果均为模拟</h1>
<p class="note">这些数值验证坐标链与评价程序，不代表 UR12e、光学定位仪或完整平台的实测精度，也不是 ASTM 符合性测试。假设机器人准确执行工具标记 M 的命令姿态；未加入机械误差、碰撞、延迟控制器或组织变形。</p>
<p>固定 seed：SEED；独立留出目标：TARGETS；场景数：SCENARIOS；几何/拒绝解析检查：CHECKS。长度 mm，角度 °。下表为每场景的留出目标均值，不能作为标准规定的统计口径。</p>
<table><thead><tr><th>场景</th><th>有效/总数</th><th>配准 FRE</th><th>真值针尖误差</th><th>估计显示误差</th><th>真值轴角误差</th><th>有效性</th></tr></thead><tbody>ROWS</tbody></table>
<h2>如何读结果</h2><p>FRE 来自参与拟合的标记点；针尖目标误差来自完全未参与拟合的目标。全部标记同向偏移时，FRE 可以接近零，真实目标误差仍为 2.5 mm。注册或工具标定被命令与显示共同使用时，显示也可能接近零，而真实针尖已经偏离。</p>
<p>动态例子指定头模速度 20 mm/s、观测滞后 0.080 s，因此独立解析位置误差为 1.600 mm。此例工具在命令位置静止，两路时间戳相同且仍在假设的新鲜度阈值内；显示不等于当前真值。遮挡、旧帧、时间不齐与头模移动使命令失效，误差记为 null，不计作 0。</p>
<p>工具轴角只比较针的 +Z 方向；完整 SO(3) 角还包含绕针轴旋转，逐目标结果保存在 <a href="run.json">run.json</a> 和 <a href="cases.csv">cases.csv</a>。独立真值在 <a href="truth.json">truth.json</a>；输入及所有固定参数在 <a href="fixture.json">fixture.json</a>。未使用任何实测硬件性能参数。</p>
<small>Truth SHA256: HASH<br>Environment: ENV</small></html>"""
    replacements = {"SEED": str(run["seed"]), "TARGETS": str(run["target_count"]),
                    "SCENARIOS": str(len(run["summary"])),
                    "CHECKS": "{}/{} 通过".format(sum(c["passed"] for c in run["checks"]), len(run["checks"])),
                    "ROWS": "\n".join(body_rows), "HASH": run["truth_sha256"],
                    "ENV": html.escape("Python {} / NumPy {}".format(sys.version.split()[0], np.__version__))}
    for key, value in replacements.items():
        document = document.replace(key, value)
    path.write_text(document, encoding="utf-8")


def run_evaluation(output, fixture=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    fixture = make_fixture() if fixture is None else copy.deepcopy(fixture)
    check_fixture(fixture)
    original_truth = copy.deepcopy(fixture["truth"])
    truth_hash = sha256(original_truth)
    # Persist truth before generating any estimate-driven command or result.
    save_json(output / "truth.json", {"schema": "ur12e_navigation_truth_v1",
        "units": "mm", "sha256": truth_hash, "truth": original_truth})
    save_json(output / "fixture.json", fixture)
    rows = [evaluate_case(fixture, scenario, target)
            for scenario, title in SCENARIOS for target in fixture["truth"]["targets_I"]]
    if sha256(fixture["truth"]) != truth_hash:
        raise AssertionError("Truth was mutated during evaluation")
    checks = oracle_checks(rows)
    checks.append({"name": "truth_unchanged", "passed": sha256(fixture["truth"]) == truth_hash})
    run = {"schema": "ur12e_navigation_run_v1", "simulation_only": True,
        "seed": fixture["seed"], "truth_sha256": truth_hash,
        "target_count": len(fixture["truth"]["targets_I"]),
        "case_count": len(rows), "summary": summarize(rows), "checks": checks,
        "all_checks_passed": all(check["passed"] for check in checks), "cases": rows,
        "environment": {"python": sys.version.split()[0], "numpy": np.__version__},
        "evidence_boundary": fixture["evidence_boundary"]}
    save_json(output / "run.json", run)
    columns = [key for key in rows[0] if key not in
               ("actual_tip_I", "display_tip_I", "goal_tip_B", "current_head_C")]
    with (output / "cases.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    write_report(output / "report.html", run)
    return run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Absolute output directory")
    parser.add_argument("--fixture", help="Optional fixture JSON from a previous run")
    args = parser.parse_args()
    if not os.path.isabs(args.output):
        parser.error("--output must be an absolute directory")
    fixture = None
    if args.fixture:
        fixture = json.loads(Path(args.fixture).read_text(encoding="utf-8"))
    run = run_evaluation(args.output, fixture)
    print(json.dumps({"output": str(Path(args.output).resolve()), "cases": run["case_count"],
                      "checks_passed": run["all_checks_passed"],
                      "truth_sha256": run["truth_sha256"]}, ensure_ascii=False))
    return 0 if run["all_checks_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
