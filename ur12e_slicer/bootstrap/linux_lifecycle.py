#!/usr/bin/env python3
"""Linux-only orchestration for the isolated mock demonstration. Never real hardware.

Only project-owned launch/Slicer processes are reused or stopped. All child
arguments are passed as lists; repository paths containing spaces are supported.
"""
import argparse
import contextlib
import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import uuid

PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
RUNTIME = PROJECT / "runtime"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix("." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def stamp():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:6]


def process(pid):
    entry = Path("/proc") / str(int(pid))
    try:
        fields = (entry / "stat").read_text().rsplit(") ", 1)[1].split()
        return {"pid": int(pid), "state": fields[0], "group": int(fields[2]),
                "start_ticks": fields[19], "command": (entry / "cmdline").read_bytes()}
    except (OSError, ValueError, IndexError):
        return None


def backend_status():
    result = subprocess.run([sys.executable, str(PROJECT / "bootstrap/check_simulation.py")],
                            capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


def slicer_status():
    result = {"running": False, "ready": False}
    try:
        owner = read_json(RUNTIME / "linux_slicer.json")
        session = read_json(RUNTIME / "slicer_bridge/session.json")
        state = read_json(RUNTIME / "slicer_bridge" / ("state_" + session["session"] + ".json"))
        proc = process(state["pid"])
        age = time.time() - state["heartbeat"]
        ours = proc and proc["state"] not in ("Z", "X") and proc["group"] == owner["group"]
        # An independently opened Slicer must not be claimed by this launcher.
        ours = ours and state.get("task") == "ur12e_slicer" and str(PROJECT).encode() in proc["command"]
        panel_ready = False
        try:
            panel = read_json(RUNTIME / "slicer_session.json")
            panel_age = time.time() - panel["heartbeat"]
            panel_ready = panel.get("pid") == state["pid"] and 0 <= panel_age < 10
        except (OSError, ValueError, KeyError):
            result["detail"] = "Control panel heartbeat is absent or invalid."
        result.update(running=bool(ours), ready=bool(ours and 0 <= age < 10 and panel_ready),
                      pid=state["pid"], heartbeat_age_s=age, group=owner["group"])
    except (OSError, ValueError, KeyError):
        pass
    return result


def wait_ready(reader, seconds, description):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        state = reader()
        if state.get("ready"):
            return state
        time.sleep(0.25)
    raise RuntimeError(description + " did not become ready; inspect runtime/linux_logs/.")


def slicer_executable():
    configured = os.environ.get("SLICER_EXECUTABLE")
    candidate = Path(configured).expanduser() if configured else PROJECT / ".local/slicer/Slicer"
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        raise RuntimeError("Linux Slicer not found. Run bootstrap/install_linux.sh or set SLICER_EXECUTABLE.")
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise RuntimeError("A Linux graphical session is required for Slicer. Use --backend-only/--no-slicer for ROS only.")
    return candidate.resolve()


def group_members(group):
    members = []
    for entry in Path("/proc").iterdir():
        if entry.name.isdigit():
            p = process(entry.name)
            if p and p["group"] == group and p["state"] not in ("Z", "X"):
                members.append(p["pid"])
    return members


def stop_slicer():
    owner_path = RUNTIME / "linux_slicer.json"
    if not owner_path.exists():
        return
    owner = read_json(owner_path)
    group = int(owner["group"])
    leader = process(owner["pid"])
    if leader and leader["start_ticks"] != owner["start_ticks"]:
        raise RuntimeError("Slicer PID was reused; no signal sent.")
    state = slicer_status()
    if state["ready"]:
        sys.path.insert(0, str(PROJECT / "slicer"))
        from task_client import send
        send("qt.QTimer.singleShot(0, slicer.app.quit)\n_result = {'quit_requested': True}", timeout=15)
    elif group_members(group):
        expected_script = str(PROJECT / "slicer/start_slicer.py").encode()
        if not leader or leader["group"] != group or expected_script not in leader["command"]:
            raise RuntimeError("Cannot verify the unresponsive Slicer launch identity; no signal sent.")
        # This group was created by this launcher; never signal by process name.
        os.killpg(group, signal.SIGTERM)
    deadline = time.monotonic() + 30
    while group_members(group):
        if time.monotonic() > deadline:
            raise RuntimeError("Owned Slicer group did not stop in 30s; logs retained, no forced kill sent.")
        time.sleep(0.2)
    owner_path.unlink(missing_ok=True)


def stop_all(backend_only=False):
    errors = []
    if not backend_only:
        try:
            stop_slicer()
        except Exception as error:
            errors.append(str(error))
    result = subprocess.run(["bash", str(PROJECT / "bootstrap/stop_simulation.sh")])
    if result.returncode:
        errors.append("Backend stop failed (exit %s)." % result.returncode)
    if errors:
        raise RuntimeError("; ".join(errors))
    print("Stopped: no live processes remain in the recorded task groups.", flush=True)


def start_all(backend_only=False):
    # Fail before starting the backend if a requested GUI cannot be launched.
    executable = None if backend_only else slicer_executable()
    logs = RUNTIME / "linux_logs"
    logs.mkdir(parents=True, exist_ok=True)
    before = backend_status()
    created_backend = not before["running"]
    created_frontend = False
    try:
        if created_backend:
            with (logs / ("backend_" + stamp() + ".log")).open("wb") as stream:
                subprocess.Popen(["bash", str(PROJECT / "bootstrap/run_simulation.sh")],
                                 cwd=REPO, stdout=stream, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, close_fds=True)
        backend = wait_ready(backend_status, 180, "ROS mock backend")
        frontend = slicer_status()
        if not backend_only and not frontend["ready"]:
            if frontend["running"]:
                raise RuntimeError("Owned Slicer is running without a fresh heartbeat; stop it before restarting.")
            env = os.environ.copy()
            # ROS/venv Qt and Python paths must not override Slicer's own runtime.
            for key in ("PYTHONPATH", "PYTHONHOME", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "LD_LIBRARY_PATH"):
                env.pop(key, None)
            with (logs / ("slicer_" + stamp() + ".log")).open("wb") as stream:
                child = subprocess.Popen([str(executable), "--no-splash", "--ignore-slicerrc", "--python-script",
                                          str(PROJECT / "slicer/start_slicer.py")],
                                         cwd=REPO, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                         stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
            created_frontend = True
            proc = process(child.pid)
            if proc is None:
                raise RuntimeError("Slicer launcher exited immediately; inspect its log.")
            write_json(RUNTIME / "linux_slicer.json", {"pid": child.pid, "group": child.pid,
                       "start_ticks": proc["start_ticks"], "executable": str(executable)})
            frontend = wait_ready(slicer_status, 120, "Slicer panel")
        print(json.dumps({"backend": backend, "slicer": frontend, "simulation_only": True}, indent=2), flush=True)
    except BaseException:
        # Only clean components created by this failed start. Reused graphs remain.
        if created_frontend:
            with contextlib.suppress(Exception):
                stop_slicer()
        if created_backend:
            subprocess.run(["bash", str(PROJECT / "bootstrap/stop_simulation.sh")])
        raise


def run_checks(args):
    identifier = stamp()
    folder = PROJECT / "output/linux_validation" / identifier
    folder.mkdir(parents=True)
    result = {"passed": False, "mode": "linux_mock_software_validation", "run_id": identifier,
              "started_at": time.time(), "offline_only": args.offline_only, "no_slicer": args.no_slicer,
              "checks": [], "boundary": "Synthetic optical observations and UR mock; no physical accuracy or ASTM conformity."}

    def check(name, command, cwd=REPO):
        path = folder / (name + ".log")
        print("Checking " + name + " ...", flush=True)
        with path.open("wb") as stream:
            completed = subprocess.run(command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT)
        result["checks"].append({"name": name, "passed": completed.returncode == 0,
                                  "exit_code": completed.returncode, "log": str(path.relative_to(PROJECT))})
        write_json(folder / "summary.json", result)
        if completed.returncode:
            raise RuntimeError(name + " failed; see " + str(path))

    locked_frontend = False
    try:
        check("geometry_tests", [sys.executable, "-m", "unittest", "discover", "-s", str(PROJECT / "navigation_benchmark"), "-p", "test_geometry.py", "-v"])
        check("bridge_tests", [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"], PROJECT / "ros2_ws/src/ur12e_slicer_bridge")
        check("resource_checks", [sys.executable, "-m", "unittest", "discover", "-s", str(PROJECT / "bootstrap"), "-p", "test_fetch_assets.py", "-v"])
        check("lifecycle_tests", [sys.executable, "-m", "unittest", "discover", "-s", str(PROJECT / "bootstrap"), "-p", "test_linux_lifecycle.py", "-v"])
        check("metrics_and_gate", [sys.executable, str(PROJECT / "validation/check_navigation_metrics.py")])
        offline = PROJECT / "output/navigation_offline" / identifier
        check("offline_evaluation", [sys.executable, str(PROJECT / "navigation_benchmark/run_offline.py"), "--output", str(offline)])
        check("point_statistics", [sys.executable, str(PROJECT / "validation/evaluate_recorded_points.py"), "--input",
                                  str(PROJECT / "research/benchmark_evidence/astm_audit/analytic_input_example.json"),
                                  "--output", str(offline / "analytic_point_statistics.json")])
        result["offline_report"] = str((offline / "report.html").relative_to(PROJECT))
        if not args.offline_only:
            start_all(backend_only=args.no_slicer)
            if not args.no_slicer:
                sys.path.insert(0, str(PROJECT / "slicer"))
                from task_client import send
                send((PROJECT / "validation/prepare_navigation_validation.py").read_text(encoding="utf-8"))
                locked_frontend = True
            check("moveit_navigation", [sys.executable, str(PROJECT / "validation/run_navigation_integration.py")])
            if not args.no_slicer:
                display = send((PROJECT / "validation/show_navigation_result.py").read_text(encoding="utf-8"), timeout=120)
                write_json(folder / "slicer_scene.json", display)
                result["checks"].append({"name": "slicer_scene", "passed": True})
                # Reopen in a separate Slicer process, as a saved scene is also a deliverable.
                check("scene_reload", [str(slicer_executable()), "--no-splash", "--ignore-slicerrc", "--no-main-window",
                                       "--python-script", str(PROJECT / "validation/verify_navigation_scene_reload.py")])
            result["integration_result"] = "output/navigation_benchmark/latest_result.json"
        result["passed"] = True
    except BaseException:
        result["error"] = traceback.format_exc()
        raise
    finally:
        if locked_frontend:
            from task_client import send
            try:
                send((PROJECT / "validation/release_navigation_validation.py").read_text(encoding="utf-8"), timeout=15)
            except Exception as error:
                result["passed"] = False
                result["unlock_error"] = str(error)
        if args.stop_after and not args.offline_only:
            try:
                stop_all(backend_only=args.no_slicer)
                result["checks"].append({"name": "stop_recorded_groups", "passed": True})
            except Exception as error:
                result["passed"] = False
                result["stop_error"] = str(error)
        result["finished_at"] = time.time()
        write_json(folder / "summary.json", result)
        write_json(PROJECT / "output/linux_validation/latest.json", result)
        print("Validation summary: " + str(folder / "summary.json"), flush=True)
    if not result["passed"]:
        raise RuntimeError("Validation cleanup failed; inspect the summary.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    start = commands.add_parser("start", help="Build/start the mock graph and Slicer, reusing healthy instances")
    start.add_argument("--backend-only", action="store_true", help="No GUI, ROS mock only")
    stop = commands.add_parser("stop", help="Stop only the recorded task's process groups")
    stop.add_argument("--backend-only", action="store_true")
    commands.add_parser("status", help="Read-only JSON readiness and heartbeat status")
    run = commands.add_parser("run", help="Run software checks, 72-case evaluation and MoveIt/Slicer validation")
    run.add_argument("--offline-only", action="store_true", help="Pure Python checks, no ROS or Slicer started")
    run.add_argument("--no-slicer", action="store_true", help="Also run MoveIt/mock, without GUI checks")
    run.add_argument("--stop-after", action="store_true", help="Stop recorded task groups after validation")
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("Use these entrypoints inside Linux; Windows .ps1 launchers remain separate.")
    if args.command == "status":
        print(json.dumps({"backend": backend_status(), "slicer": slicer_status(), "simulation_only": True}, indent=2))
        return
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / "linux_lifecycle.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Another Linux lifecycle/validation command owns this project; wait for it to finish.")
        if args.command == "start":
            start_all(args.backend_only)
        elif args.command == "stop":
            stop_all(args.backend_only)
        else:
            run_checks(args)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        sys.exit(1)
