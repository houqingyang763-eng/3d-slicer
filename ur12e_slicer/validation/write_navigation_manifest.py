"""Freeze hashes and verification references for an already completed delivery."""
import datetime
import hashlib
import json
from pathlib import Path
import platform
import zipfile

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
OUTPUT = PROJECT / "output" / "navigation_benchmark"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    result = json.loads((OUTPUT / "latest_result.json").read_text(encoding="utf-8"))
    bundle = json.loads((OUTPUT / "latest_bundle.json").read_text(encoding="utf-8-sig"))
    offline_path = Path(bundle["offline_report"]).parent / "run.json"
    offline = json.loads(offline_path.read_text(encoding="utf-8"))
    metrics = json.loads((OUTPUT / "metric_and_gate_checks.json").read_text(encoding="utf-8"))
    reload_result = json.loads((OUTPUT / "scene_reload_verification.json").read_text(encoding="utf-8"))
    assert result["passed"] and offline["all_checks_passed"] and metrics["passed"] and reload_result["passed"]
    code_paths = sorted((PROJECT / "navigation_benchmark").glob("*.py"))
    code_paths += [PROJECT / "validation" / name for name in (
        "run_navigation_integration.py", "check_navigation_metrics.py", "evaluate_recorded_points.py",
        "show_navigation_result.py", "verify_navigation_scene_reload.py")]
    code_paths += [PROJECT / "slicer" / "navigation_benchmark_scene.py", PROJECT / "Run_Navigation_Benchmark.ps1"]
    with zipfile.ZipFile(OUTPUT / "latest_scene.mrb") as package:
        members = package.namelist()
    manifest = {"schema_version": 1, "created_at": datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
        "simulation_only": True, "astm_conformity_claim": False,
        "integration_run_id": result["run_id"], "offline_run": str(offline_path.parent),
        "integration_truth_sha256": result["truth_sha256"], "offline_truth_sha256": offline["truth_sha256"],
        "verified": {"offline_case_count": offline["case_count"], "offline_analytic_checks": len(offline["checks"]),
            "offline_passed": True, "metric_and_gate_checks": len(metrics["checks"]),
            "executed_cases": len(result["cases"]), "rejected_cases": len(result["rejection_cases"]),
            "scene_reopened": True, "mrb_member_count": len(members),
            "target_inside_nonzero_MR": reload_result["target_inside_volume"]},
        "geometry_unit_test_note": "26 unit tests passed in Slicer Python 3.12.10 / NumPy 2.4.6; runner stdout recorded in task tools. Earlier independent run verification.json uses bundled Python.",
        "manifest_environment": {"python": platform.python_version(), "numpy": np.__version__},
        "code_sha256": {str(path.relative_to(PROJECT)).replace("\\", "/"): sha(path) for path in code_paths},
        "artifact_sha256": {name: sha(OUTPUT / name) for name in ("latest_result.json", "latest_scene.mrb", "latest_scene.png", "latest_platform.png", "latest_mri_slices.png", "scene_reload_verification.json")},
        "MR_source_sha256": sha(PROJECT.parent / "data" / "MR-head.nrrd"),
        "limitations": result["limitations"], "saved_scene_members": members}
    destination = OUTPUT / "delivery_manifest.json"
    destination.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"manifest": str(destination), "verified": manifest["verified"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
