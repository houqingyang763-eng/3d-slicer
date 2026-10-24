"""Analytic oracles for statistics and navigation authorization, not device tests."""
import copy
import json
from pathlib import Path
import sys

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from navigation_benchmark.core import ObservationRejected, matrix_from_pose
from navigation_benchmark.gateway import NavigationGate
from navigation_benchmark.metrics import pairwise_distance_statistics, point_statistics


def run():
    checks = []
    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        checks.append({"id": name, "passed": True})
    symmetric = point_statistics([[-1, 0, 0], [1, 0, 0]], [0, 0, 0])
    check("bias_zero_does_not_mean_zero_error", symmetric["bias_norm_mm"] == 0 and symmetric["rms_distance_mm"] == 1 and symmetric["mean_distance_mm"] == 1 and symmetric["precision_rms_mm"] == 1)
    fixed = point_statistics([[3, 4, 0], [3, 4, 0]], [0, 0, 0])
    check("constant_bias_five_mm_zero_jitter", fixed["bias_norm_mm"] == 5 and fixed["precision_rms_mm"] == 0)
    empty = point_statistics([[0, 0, 0]], [0, 0, 0], [False])
    check("missing_is_null_not_zero", empty["rms_distance_mm"] is None and empty["n_invalid"] == 1)
    zero = point_statistics([[0, 0, 0]], [0, 0, 0])
    check("valid_origin_is_not_missing", zero["n_valid"] == 1 and zero["mean_distance_mm"] == 0)
    pair = pairwise_distance_statistics([[10, 20, 30], [13, 24, 30], [10, 20, 42]], [[0, 0, 0], [3, 4, 0], [0, 0, 12]])
    check("all_pairs_translation_invariant", pair["pair_count"] == 3 and pair["max_abs_distance_error_mm"] == 0)
    varied = point_statistics([[1, 0, 0], [2, 1, 0]], [[0, 0, 0], [1, 0, 0]])
    check("multi_target_spread_is_not_fixed_point_precision", varied["constant_truth_point"] is False and varied["precision_rms_mm"] is None and varied["error_vector_dispersion_rms_mm"] is not None)
    cal = {key: np.eye(4) for key in ("T_B_C", "T_H_I", "T_M_Tip")}
    obs = {"now_s": 10.0, "head": {"matrix": np.eye(4), "timestamp_s": 10.0, "valid": True, "clock_domain": "unix_seconds"}, "tool": {"matrix": np.eye(4), "timestamp_s": 10.0, "valid": True, "clock_domain": "unix_seconds"}}
    gate = NavigationGate(clock=lambda: 10.0)
    context = gate.plan_context(cal, obs, np.eye(4))
    for name, expected in (("calibration", "calibration_changed"), ("target", "target_changed"), ("head", "head_moved")):
        changed_cal, changed_obs, target = copy.deepcopy(cal), copy.deepcopy(obs), np.eye(4)
        if name == "calibration": changed_cal["T_B_C"][0, 3] = 1
        if name == "target": target[0, 3] = 1
        if name == "head": changed_obs["head"]["matrix"][0, 3] = 5
        try:
            gate.authorize_execute(context, changed_cal, changed_obs, target)
        except ObservationRejected as exc:
            check(name + "_invalidates_plan", exc.code == expected)
        else:
            raise AssertionError(name + " was accepted")
    replayed = copy.deepcopy(obs)
    replayed["now_s"] = 9.0
    replayed["head"]["timestamp_s"] = replayed["tool"]["timestamp_s"] = 9.0
    try:
        gate.authorize_execute(context, cal, replayed, np.eye(4))
    except ObservationRejected as exc:
        check("whole_packet_replay_cannot_renew_itself", exc.code == "stale_frame")
    else:
        raise AssertionError("Replayed packet was accepted")
    result = {"passed": True, "mode": "analytic_software_checks_not_ASTMid_or_hardware", "checks": checks, "analytic_examples": {"symmetric": symmetric, "fixed_bias": fixed, "missing": empty, "pairs": pair}}
    output = PROJECT / "output" / "navigation_benchmark" / "metric_and_gate_checks.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"passed": True, "checks": len(checks), "output": str(output)}))


if __name__ == "__main__":
    run()
