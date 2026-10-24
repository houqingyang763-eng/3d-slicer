#!/usr/bin/env python3
"""Pause only the owned mock file bridge; verify real Slicer stale/recovery UI.

SIGSTOP interrupts actual state-file updates without fabricating state.json.
The recorded process is resumed in finally. The running controller and joint
publisher are independently observed and never signalled by this script.
Run in the task's WSL environment with an exclusive validation window.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
import traceback
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from scene_tool import atomic_json

PROJECT = Path(__file__).resolve().parents[1]


def signature(path):
    if not path.exists():
        return None
    return {"mtime_ns": path.stat().st_mtime_ns, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def proc_identity(pid):
    proc = Path(f"/proc/{pid}")
    stat = proc.joinpath("stat").read_text().split(") ", 1)[1].split()
    command = proc.joinpath("cmdline").read_bytes().split(b"\0")
    return {"pid": pid, "start_ticks": stat[19], "pgid": os.getpgid(pid),
            "command": [part.decode(errors="replace") for part in command if part]}


def find_bridge(runtime):
    owner = json.loads((runtime / "linux_launch.json").read_text(encoding="utf-8"))
    launch = proc_identity(int(owner["pid"]))
    assert launch["start_ticks"] == str(owner["start_ticks"]) and launch["pgid"] == launch["pid"], "Recorded launch PID identity changed"
    assert "ur12e_slicer_bridge" in " ".join(launch["command"]), "Recorded group does not belong to this mock launch"
    matches = []
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            current = proc_identity(int(path.name))
            if (current["pgid"] == launch["pgid"] and
                    any(part.endswith("/file_bridge") and "ur12e_slicer_bridge" in part for part in current["command"])):
                matches.append(current)
        except (OSError, ValueError):
            pass
    assert len(matches) == 1, f"Expected one file bridge in the recorded group, found {len(matches)}"
    return launch, matches[0]


def slicer_snapshot(relative, reject_command=False):
    sys.path.insert(0, str(PROJECT / "slicer"))
    from task_client import send
    code = ("import time\n"
            "p=slicer.ur12eSimulation\np.poll()\n"
            "_result={'observed_at':time.time(),'status_text':str(p.status_label.text),"
            "'error_text':str(p.error_label.text),'pose_text':str(p.pose_label.text),'buttons':{k:bool(b.isEnabled()) for k,b in p.submit_buttons},"
            "'tool_opacity':float(p.tool.GetDisplayNode().GetOpacity()),'state':p.state}\n"
            f"_result['screenshot']=p.capture(PROJECT/{str(relative)!r})\n")
    if reject_command:
        code += "_result['stale_plan_send_result']=p.send('plan')\n_result['refusal_text']=str(p.status_label.text)\n"
    response = send(code, timeout=20)
    return response["result"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=PROJECT / "runtime")
    parser.add_argument("--output", type=Path, default=PROJECT / "output" / "feedback_disconnect")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "62" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Use ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1")
    runtime = args.runtime.resolve()
    folder = args.output.resolve() / (time.strftime("%Y%m%d_%H%M%S")+"_"+uuid.uuid4().hex[:6])
    folder.mkdir(parents=True)
    report = {"passed": False, "simulation_only": True, "started_at": time.time(),
              "evidence_type": "real_file_bridge_feedback_interruption_in_real_slicer",
              "interruption_method": "SIGSTOP only the verified owned file_bridge; SIGCONT in finally",
              "scope": "Verifies stale file-bridge feedback handling, not a network cable or hardware disconnection."}
    state_file, target_file = runtime / "state.json", runtime / "target.json"
    lock = runtime / "validation.lock"
    owns_lock, stopped, bridge, node = False, False, None, None
    observed = []
    rclpy.init()
    try:
        with lock.open("x", encoding="utf-8") as handle:
            json.dump({"pid": os.getpid(), "scope": "feedback_disconnect", "created_at": time.time()}, handle)
        owns_lock = True
        state = json.loads(state_file.read_text(encoding="utf-8"))
        assert state.get("source") == "ros2_mock_hardware" and state.get("ready") and state.get("feedback_valid"), "Genuine ready mock feedback required"
        assert time.time()-state["timestamp"] < 1 and state.get("status") not in ("planning", "executing"), "Backend must be fresh and stationary"
        launch, bridge = find_bridge(runtime)
        report["launch_identity"], report["bridge_identity"] = launch, bridge
        report["before_ui"] = slicer_snapshot((folder / "01_connected.png").relative_to(PROJECT).as_posix())
        assert report["before_ui"]["buttons"].get("plan"), "Connected panel did not enable planning"
        target_before = signature(target_file)
        node = rclpy.create_node("ur12e_disconnect_joint_observer")
        def on_joints(message):
            observed.append({"received_at": time.time(), "joint_names": list(message.name), "joint_positions_rad": list(message.position)})
        node.create_subscription(JointState, "/joint_states", on_joints, qos_profile_sensor_data)
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert observed, "Direct ROS joint-state observer received nothing"
        assert proc_identity(bridge["pid"])["start_ticks"] == bridge["start_ticks"], "File bridge PID identity changed before pausing"
        os.kill(bridge["pid"], signal.SIGSTOP)
        stopped = True
        time.sleep(.2)
        stopped_signature = signature(state_file)
        report["feedback_stop_signature"] = stopped_signature
        report["paused_at"] = time.time()
        start_index = len(observed)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        report["during_ui"] = slicer_snapshot((folder / "02_feedback_interrupted.png").relative_to(PROJECT).as_posix(), reject_command=True)
        report["feedback_remained_unchanged"] = signature(state_file) == stopped_signature
        report["target_unchanged_after_refused_send"] = signature(target_file) == target_before
        during = report["during_ui"]
        report["checks"] = {
            "state_file_actually_stopped_updating": report["feedback_remained_unchanged"],
            "visible_interruption_message": "反馈已中断" in during["status_text"],
            "last_error_explicitly_marked_stale": "末次反馈（已过期）" in during["error_text"],
            "last_tip_pose_explicitly_marked_invalid": "末次针尖 RAS（反馈无效）" in during["pose_text"],
            "all_motion_commands_disabled": all(not enabled for enabled in during["buttons"].values()),
            "last_robot_display_dimmed": during["tool_opacity"] <= .21,
            "stale_send_refused_without_file_write": during["stale_plan_send_result"] is None and report["target_unchanged_after_refused_send"],
            "other_ros_joint_feedback_continued": len(observed)-start_index >= 10,
        }
        report["direct_joint_samples_during_pause"] = len(observed)-start_index
        assert all(report["checks"].values()), "Slicer did not pass every interruption check"
    except Exception as exc:
        report.update({"error": str(exc), "traceback": traceback.format_exc()})
    finally:
        if stopped:
            try:
                current = proc_identity(bridge["pid"])
                assert current["start_ticks"] == bridge["start_ticks"] and current["command"] == bridge["command"], "Paused PID identity changed; refusing a signal to an unrelated process"
                os.kill(bridge["pid"], signal.SIGCONT)
                stopped = False
                report["resumed_at"] = time.time()
                report["same_bridge_resumed"] = True
            except Exception:
                report["resume_error"] = traceback.format_exc()
        if report.get("same_bridge_resumed"):
            try:
                deadline = time.monotonic()+15
                while time.monotonic() < deadline:
                    state = json.loads(state_file.read_text(encoding="utf-8"))
                    if state.get("ready") and state.get("feedback_valid") and time.time()-state["timestamp"] < 1 and state["timestamp"] > report["during_ui"]["state"]["timestamp"]:
                        break
                    if node:
                        rclpy.spin_once(node, timeout_sec=.1)
                    else:
                        time.sleep(.1)
                report["after_ui"] = slicer_snapshot((folder / "03_feedback_recovered.png").relative_to(PROJECT).as_posix())
                after = report["after_ui"]
                recovered = (after["state"].get("ready") and after["state"].get("feedback_valid") and after["buttons"].get("plan") and "反馈已中断" not in after["status_text"] and "末次反馈" not in after["error_text"] and "反馈无效" not in after["pose_text"])
                report.setdefault("checks", {})["same_bridge_feedback_and_panel_recovered"] = bool(recovered)
                report["passed"] = bool(not report.get("error") and all(report["checks"].values()))
            except Exception:
                report["recovery_error"] = traceback.format_exc()
        report["finished_at"] = time.time()
        atomic_json(folder / "direct_joint_observations.json", observed)
        atomic_json(folder / "results.json", report)
        atomic_json(PROJECT / "output" / "feedback_disconnect_status.json", {"passed": report["passed"], "directory": str(folder), "timestamp": time.time()})
        if owns_lock:
            lock.unlink()
        if node:
            node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"passed": report["passed"], "output": str(folder), "error": report.get("error"), "checks": report.get("checks")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
