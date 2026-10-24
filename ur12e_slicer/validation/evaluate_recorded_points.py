"""Evaluate an explicit static-point JSON; no ASTM compliance assertion.

Input matches research/benchmark_evidence/astm_audit/analytic_input_example.json.
Replace its coordinates with independently measured physical references and
record actual conditions before using this adapter for physical experiments.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from navigation_benchmark.metrics import point_statistics


def evaluate(source):
    if source.get("units", {}).get("position") != "mm":
        raise ValueError("Explicit millimetre input required")
    samples = source["samples"]
    measured = np.asarray([sample["p_mm"] if sample.get("valid") else [np.nan]*3
                           for sample in samples], dtype=float).reshape(-1, 3)
    flags = [sample.get("valid") is True and sample.get("status") == "OK" for sample in samples]
    result = point_statistics(measured, source["target"]["q_mm"], flags)
    result["n_expected"] = source["expected_acquisitions"]
    if result["n_expected"] < result["n_total"]:
        raise ValueError("More samples than the declared acquisition count")
    result["n_not_acquired"] = result["n_expected"]-result["n_total"]
    result["source_target"] = source["target"]
    result["conditions"] = source.get("conditions", {})
    result["evidence_boundary"] = "Statistics only; physical protocol and independent reference must be validated separately"
    result.update({"mean_point_error_mm": result["mean_distance_mm"],
                   "point_rmse_mm": result["rms_distance_mm"],
                   "max_point_error_mm": result["max_distance_mm"]})
    if "expected" in source:
        checks = {}
        for key, expected in source["expected"].items():
            checks[key] = result.get(key) is not None and bool(np.allclose(result[key], expected, atol=1e-10, rtol=0))
        result["analytic_expected_checks"] = checks
        result["analytic_expected_passed"] = all(checks.values())
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(json.loads(args.input.read_text(encoding="utf-8-sig")))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(args.output), "n_valid": result["n_valid"],
                      "analytic_expected_passed": result.get("analytic_expected_passed")}))
    sys.exit(0 if result.get("analytic_expected_passed", True) else 1)
