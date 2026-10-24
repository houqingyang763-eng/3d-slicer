#!/usr/bin/env python3
"""Read-only, network-free Ubuntu/ROS/resource readiness check; starts no nodes."""
import argparse
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
from fetch_assets import PROJECT, read_json, resolve_slicer, restore_description, restore_mr, restore_slicer


def check(skip_slicer=False):
    checks = []
    warnings = []

    def record(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed), "detail": str(detail)})

    release = {}
    try:
        for line in Path("/etc/os-release").read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                release[key] = value.strip('"')
    except OSError:
        pass
    record("os", platform.system() == "Linux" and release.get("ID") == "ubuntu" and release.get("VERSION_ID") == "24.04",
           "Supported target: Ubuntu 24.04 x86_64; found " + platform.platform())
    record("architecture", platform.machine() == "x86_64", platform.machine())
    record("python", sys.version_info >= (3, 12), sys.version.split()[0])
    for executable in ("bash", "colcon", "setsid"):
        record("command:" + executable, shutil.which(executable) is not None, shutil.which(executable) or "missing")
    for module in ("numpy", "yaml"):
        try:
            imported = __import__(module)
            record("python:" + module, True, getattr(imported, "__version__", "available"))
        except ImportError as error:
            record("python:" + module, False, error)
    record("ROS setup", Path("/opt/ros/jazzy/setup.bash").is_file(), "/opt/ros/jazzy/setup.bash")
    required = ["rclpy", "ur_robot_driver", "ur_moveit_config", "ur_description", "moveit_ros_move_group", "moveit_msgs", "controller_manager"]
    for package in required:
        location = Path("/opt/ros/jazzy/share") / package
        record("ROS package:" + package, location.is_dir(), location)
    for relative in ("ur_robot_driver/launch/ur_control.launch.py", "ur_moveit_config/launch/ur_moveit.launch.py"):
        path = Path("/opt/ros/jazzy/share") / relative
        record("UR12e support:" + relative, path.is_file() and "ur12e" in path.read_text(), path)
    reference = read_json(PROJECT / "assets" / "validated_linux_versions.json")
    for package, expected in reference["observed_debian_versions"].items():
        try:
            current = subprocess.check_output(["dpkg-query", "-W", "-f=${Version}", package], text=True, stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError):
            current = "missing"
        record("installed:" + package, current != "missing", current)
        if current != expected and current != "missing":
            warnings.append("%s=%s differs from verified reference %s; run the provided runtime acceptance tests." % (package, current, expected))
    for name, function in (("UR description", restore_description), ("MR image", restore_mr)):
        try:
            record(name, True, function(check_only=True))
        except (OSError, RuntimeError, ValueError) as error:
            record(name, False, error)
    if not skip_slicer:
        try:
            record("Slicer", True, restore_slicer(check_only=True))
            if os.environ.get("SLICER_EXECUTABLE"):
                warnings.append("SLICER_EXECUTABLE is operator-managed; this check does not confirm its version or archive hash.")
        except (OSError, RuntimeError, ValueError) as error:
            record("Slicer", False, error)
        for package in reference["slicer_ubuntu_packages"]:
            try:
                status = subprocess.check_output(["dpkg-query", "-W", "-f=${Status}", package], text=True, stderr=subprocess.DEVNULL).strip()
            except (OSError, subprocess.CalledProcessError):
                status = "missing"
            record("GUI dependency:" + package, status == "install ok installed", status)
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            warnings.append("No DISPLAY/WAYLAND_DISPLAY in this shell. GUI launch needs a desktop session; offline/ROS checks can still run.")
    warnings.append("Package presence and asset checks do not establish successful planning, execution, GUI rendering or physical-device accuracy.")
    return {"passed": all(item["passed"] for item in checks), "checks": checks, "warnings": warnings,
            "slicer_executable": str(resolve_slicer()) if not skip_slicer else None,
            "version_policy": reference["policy"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-slicer", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    result = check(args.skip_slicer)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        for item in result["checks"]:
            print(("OK   " if item["passed"] else "FAIL ") + item["name"] + ": " + item["detail"])
        for warning in result["warnings"]:
            print("NOTE " + warning)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
