#!/usr/bin/env python3
"""Exercise the running ROS mock backend through the public file protocol.

This is an integration gate, never an emulator. Start the approved mock launch
first, and keep the Slicer UI from sending commands until this script exits.
Only Python's standard library is required on either Windows or WSL.
"""
import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time
import traceback
import uuid


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_LINKS = {"base_link", "shoulder_link", "upper_arm_link", "forearm_link",
                  "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "needle_tip"}
BUSY = {"waiting_for_robot", "planning", "executing"}


def atomic_json(path, value):
    descriptor, temporary = tempfile.mkstemp(prefix=".validation-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, allow_nan=False, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def vector(value, size):
    assert isinstance(value, list) and len(value) == size, f"Expected {size}-vector: {value}"
    assert all(type(x) in (int, float) and math.isfinite(x) for x in value), "Nonfinite vector"
    return value


def normalized(q):
    vector(q, 4)
    norm = math.sqrt(sum(x * x for x in q))
    assert norm > 1e-9, "Zero quaternion"
    return [x / norm for x in q]


def pose_error(actual, target):
    a, b = vector(actual["position_m"], 3), vector(target["position_m"], 3)
    qa, qb = normalized(actual["orientation_xyzw"]), normalized(target["orientation_xyzw"])
    distance = math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return distance, 2 * math.acos(min(1.0, dot))


def quaternion_product(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return [w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
            w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s]


def joint_delta(before, after):
    a = dict(zip(before["joint_names"], before["joint_positions_rad"]))
    b = dict(zip(after["joint_names"], after["joint_positions_rad"]))
    assert set(a) == set(b), "Joint names changed during validation"
    return max(abs(a[key] - b[key]) for key in a)


class Validation:
    def __init__(self, args):
        self.args = args
        self.runtime = args.runtime.resolve()
        self.state_path = self.runtime / "state.json"
        self.event_path = self.runtime / "ros_events.jsonl"
        self.run_id = time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
        self.output = args.output.resolve() / self.run_id
        self.output.mkdir(parents=True, exist_ok=False)
        self.summary = {"schema_version": 1, "run_id": self.run_id, "started_at": time.time(),
                        "mode": "real_ros2_mock_hardware_integration", "passed": False,
                        "limits": {"position_error_m": args.position_tolerance,
                                   "orientation_error_rad": args.orientation_tolerance,
                                   "max_feedback_age_s": args.max_feedback_age}, "cases": []}
        self.event_offset = self.event_path.stat().st_size if self.event_path.exists() else 0
        self.trace = (self.output / "observations.jsonl").open("w", encoding="utf-8")
        self.last_trace_timestamp = None

    def read_state(self):
        state = json.loads(self.state_path.read_text(encoding="utf-8-sig"))
        now = time.time()
        assert state.get("source") == "ros2_mock_hardware", "Refusing non-ROS backend"
        assert state.get("schema_version") == 1, "Unexpected state schema"
        stamp = state.get("timestamp")
        assert type(stamp) in (int, float) and math.isfinite(stamp), "Missing state timestamp"
        assert -0.5 <= now - stamp <= self.args.max_feedback_age, "Whole backend state is stale"
        assert now - self.state_path.stat().st_mtime <= self.args.max_feedback_age, "State file is stale"
        assert state.get("feedback_valid") is True, "Robot feedback is invalid"
        for field in ("joint_state_age_s", "tf_age_s"):
            age = state.get(field)
            assert type(age) in (int, float) and 0 <= age < self.args.max_feedback_age, f"Stale {field}: {age}"
        names = state.get("joint_names", [])
        assert len(names) == 6 and len(set(names)) == 6, "Six named joints are required"
        vector(state.get("joint_positions_rad"), 6)
        assert state.get("needle_collision_ready") is True, "Needle is absent from planning scene"
        assert REQUIRED_LINKS <= set(state.get("link_transforms", {})), "Incomplete TF for Slicer robot display"
        tcp = state.get("tcp_pose")
        assert isinstance(tcp, dict) and tcp.get("frame_id") == "base", "Invalid needle-tip frame"
        vector(tcp.get("position_m"), 3)
        normalized(tcp.get("orientation_xyzw"))
        tip_distance, tip_angle = pose_error(tcp, state["link_transforms"]["needle_tip"])
        assert tip_distance < 1e-9 and tip_angle < 1e-7, "Needle TF / TCP mismatch"
        if stamp != self.last_trace_timestamp:
            observed = {key: state.get(key) for key in (
                "timestamp", "request_id", "status", "message", "ready", "source", "feedback_valid",
                "joint_state_age_s", "tf_age_s", "joint_names", "joint_positions_rad", "tcp_pose",
                "position_error_m", "orientation_error_rad", "execution")}
            self.trace.write(json.dumps(observed, allow_nan=False) + "\n")
            self.trace.flush()
            self.last_trace_timestamp = stamp
        return state

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        last_error = "No state file"
        while time.monotonic() < deadline:
            try:
                state = self.read_state()
                if state.get("ready") and state.get("status") not in BUSY:
                    return state
            except (OSError, ValueError, AssertionError) as exc:
                last_error = str(exc)
            time.sleep(0.1)
        raise AssertionError(f"ROS backend not ready within {self.args.ready_timeout}s: {last_error}")

    def submit(self, command, pose, label, plan_id=None):
        request = {"schema_version": 1, "request_id": f"gate-{self.run_id}-{label}-{uuid.uuid4().hex[:6]}",
                   "command": command, "frame_id": "base", "position_m": list(pose["position_m"]),
                   "orientation_xyzw": list(pose["orientation_xyzw"]), "timestamp": time.time()}
        if plan_id:
            request["plan_request_id"] = plan_id
        atomic_json(self.output / f"{label}_request.json", request)
        atomic_json(self.runtime / "target.json", request)
        print(f"Submitted {label}: {command} ({request['request_id']})", flush=True)
        return request

    def wait_terminal(self, request):
        deadline = time.monotonic() + self.args.step_timeout
        observed_statuses = []
        states = []
        while time.monotonic() < deadline:
            # Once ready, stale feedback is a failure, not silently retried.
            state = self.read_state()
            if state.get("request_id") == request["request_id"]:
                states.append(state)
                if not observed_statuses or observed_statuses[-1] != state["status"]:
                    observed_statuses.append(state["status"])
                if state["status"] in {"planned", "succeeded", "error"}:
                    return state, observed_statuses, states
            if state.get("last_rejected_request_id") == request["request_id"]:
                raise AssertionError("Backend rejected integration request (another sender may be active)")
            time.sleep(0.05)
        raise AssertionError(f"Timed out waiting for ROS result: {request['request_id']}; {observed_statuses}")

    def wait_preview(self, request_id, observations=None):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            state = self.read_state()
            if observations is not None:
                observations.append(state)
            plan = state.get("plan", {})
            assert plan.get("request_id") == request_id, "Plan replaced by another sender"
            if len(plan.get("tcp_path_m", [])) >= 2:
                return state
            time.sleep(0.1)
        raise AssertionError("Actual MoveIt FK trajectory preview did not arrive")

    def settle(self, request_id, duration=0.5, observations=None):
        deadline = time.monotonic() + duration
        state = None
        while time.monotonic() < deadline:
            state = self.read_state()
            if observations is not None:
                observations.append(state)
            assert state.get("request_id") == request_id, "Another sender replaced active result"
            time.sleep(0.05)
        return state

    def save_case(self, label, request, state, **details):
        atomic_json(self.output / f"{label}_state.json", state)
        result = {"name": label, "request_id": request["request_id"], "command": request["command"],
                  "status": state["status"], "message": state.get("message"),
                  "plan": {k: state.get("plan", {}).get(k) for k in (
                      "request_id", "status", "point_count", "planning_time_s", "moveit_error_code", "preview_message")},
                  "execution": state.get("execution"), **details}
        self.summary["cases"].append(result)
        atomic_json(self.output / "summary.json", self.summary)
        return result

    def check_execution(self, label, request, before, state, statuses, observations):
        assert state["status"] == "succeeded", f"{label}: {state['message']}"
        execution = state.get("execution", {})
        assert execution.get("status") == "succeeded", "No successful execution result"
        assert execution.get("moveit_error_code") == 1 and execution.get("action_status") == 4, "ROS action did not succeed"
        state = self.wait_preview(state["plan"]["request_id"], observations)
        state = self.settle(request["request_id"], observations=observations)
        distance, angle = pose_error(state["tcp_pose"], request)
        displacement = joint_delta(before, state)
        observed_motion = max(joint_delta(before, observed) for observed in observations + [state])
        case = self.save_case(label, request, state, observed_statuses=statuses,
                              independent_position_error_m=distance, independent_orientation_error_rad=angle,
                              final_joint_change_rad=displacement, observed_joint_change_rad=observed_motion,
                              fk_preview_points=len(state["plan"]["tcp_path_m"]), passed=False)
        assert observed_motion > 1e-4, "No observed joint movement despite displaced goal"
        assert abs(distance - state["position_error_m"]) < 1e-8, "Backend position error disagrees with independent calculation"
        assert abs(angle - state["orientation_error_rad"]) < 1e-7, "Backend orientation error disagrees with independent calculation"
        assert distance <= self.args.position_tolerance, f"Tip position error {distance * 1000:.4f} mm exceeds budget"
        assert angle <= self.args.orientation_tolerance, f"Orientation error {angle:.6f} rad exceeds budget"
        case["passed"] = True
        print(f"PASS {label}: tip {distance*1000:.4f} mm, angle {angle:.6f} rad, joint change {observed_motion:.5f} rad", flush=True)
        return state

    def run(self):
        initial = self.wait_ready()
        atomic_json(self.output / "initial_state.json", initial)
        self.summary["initial_tcp_pose"] = initial["tcp_pose"]

        # Reachable candidates are close to observed actual TCP, retaining its
        # orientation; no hard-coded example pose is assumed to be reachable.
        # Use observed feedback from this demo's bent-elbow home, rather than
        # relying on the official default straight-elbow configuration.
        pose = {"position_m": [x + d for x, d in zip(initial["tcp_pose"]["position_m"], (0, 0, -0.025))],
                "orientation_xyzw": initial["tcp_pose"]["orientation_xyzw"]}
        request = self.submit("plan", pose, "01_plan_only")
        planned, statuses, observed = self.wait_terminal(request)
        assert planned["status"] == "planned", f"Plan-only failed: {planned['message']}"
        assert planned["plan"].get("moveit_error_code") == 1 and planned["plan"]["point_count"] >= 2, "No usable actual ROS plan"
        planned = self.wait_preview(request["request_id"], observed)
        planned = self.settle(request["request_id"], observations=observed)
        no_motion = max(joint_delta(initial, s) for s in observed + [planned])
        self.save_case("01_plan_only", request, planned, observed_statuses=statuses,
                       max_joint_change_rad=no_motion, fk_preview_points=len(planned["plan"]["tcp_path_m"]),
                       passed=no_motion < 1e-5 and planned["execution"]["status"] == "none")
        assert no_motion < 1e-5 and planned["execution"]["status"] == "none", "Plan-only moved the robot"

        execute = self.submit("execute", pose, "02_execute_cached", request["request_id"])
        state, statuses, observations = self.wait_terminal(execute)
        assert state["plan"]["request_id"] == request["request_id"], "Execution did not reference the cached plan"
        state = self.check_execution("02_execute_cached", execute, planned, state, statuses, observations)

        # Verify a consumed plan cannot be replayed.
        replay = self.submit("execute", pose, "03_reject_replay", request["request_id"])
        failed, statuses, observations = self.wait_terminal(replay)
        failed = self.settle(replay["request_id"], observations=observations)
        movement = max(joint_delta(state, s) for s in observations + [failed])
        self.save_case("03_reject_replay", replay, failed, observed_statuses=statuses,
                       max_joint_change_rad=movement, passed=failed["status"] == "error" and movement < 1e-5)
        assert failed["status"] == "error" and "cached plan" in failed["message"].lower() and movement < 1e-5, "Consumed cached plan was not rejected safely"

        for label, delta, orientation_delta in (("04_move_lateral", (0.020, 0, 0), 0.0),
                                                 ("05_move_and_orient", (0, -0.020, 0.010), 0.05)):
            before = self.wait_ready()
            current = before["tcp_pose"]
            q = current["orientation_xyzw"]
            if orientation_delta:
                q = quaternion_product(q, [0, 0, math.sin(orientation_delta/2), math.cos(orientation_delta/2)])
            pose = {"position_m": [x+d for x, d in zip(current["position_m"], delta)], "orientation_xyzw": q}
            request = self.submit("plan_execute", pose, label)
            state, statuses, observations = self.wait_terminal(request)
            self.check_execution(label, request, before, state, statuses, observations)

        before = self.wait_ready()
        unreachable = {"position_m": [5.0, 0.0, 5.0], "orientation_xyzw": before["tcp_pose"]["orientation_xyzw"]}
        request = self.submit("plan_execute", unreachable, "06_unreachable")
        state, statuses, observations = self.wait_terminal(request)
        state = self.settle(request["request_id"], observations=observations)
        movement = max(joint_delta(before, s) for s in observations + [state])
        plan = state.get("plan", {})
        rejected = state["status"] == "error" and plan.get("status") == "failed" and plan.get("moveit_error_code") not in (None, 1)
        self.save_case("06_unreachable", request, state, observed_statuses=statuses,
                       max_joint_change_rad=movement, passed=rejected and movement < 1e-5)
        assert rejected, "Unreachable target must return an actual MoveIt planning failure"
        assert state["execution"]["status"] == "none" and movement < 1e-5, "Unreachable target triggered motion"

        # Leave a valid final target/result for the viewer after the negative case.
        before = self.wait_ready()
        restore = {"position_m": [x+d for x, d in zip(before["tcp_pose"]["position_m"], (0, 0, -0.010))],
                   "orientation_xyzw": before["tcp_pose"]["orientation_xyzw"]}
        request = self.submit("plan_execute", restore, "07_final_reachable")
        state, statuses, observations = self.wait_terminal(request)
        self.check_execution("07_final_reachable", request, before, state, statuses, observations)

        events = self.capture_events()
        parsed = [json.loads(line) for line in events.splitlines() if line.strip()]
        for case in self.summary["cases"]:
            own = [event for event in parsed if event.get("request_id") == case["request_id"]]
            assert any(e["event"] == "request_accepted" for e in own), "Missing backend accepted event"
            if case["command"] in ("execute", "plan_execute") and case["status"] == "succeeded":
                assert any(e["event"] == "execution_result" and e.get("action_status") == 4 and e.get("moveit_error_code") == 1 for e in own), "Missing real ROS execution evidence"
        self.summary["passed"] = True

    def capture_events(self):
        events = self.event_path.read_bytes()[self.event_offset:].decode("utf-8") if self.event_path.exists() else ""
        (self.output / "ros_events.jsonl").write_text(events, encoding="utf-8")
        return events

    def finish(self):
        self.summary["finished_at"] = time.time()
        self.summary["elapsed_s"] = self.summary["finished_at"] - self.summary["started_at"]
        self.capture_events()
        try:
            last_state = json.loads(self.state_path.read_text(encoding="utf-8-sig"))
            atomic_json(self.output / "last_state.json", last_state)
        except (OSError, ValueError):
            pass
        atomic_json(self.output / "summary.json", self.summary)
        self.trace.close()
        print(f"Results: {self.output}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=ROOT / "runtime")
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "ros_runtime_validation")
    parser.add_argument("--ready-timeout", type=float, default=60)
    parser.add_argument("--step-timeout", type=float, default=150)
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance", type=float, default=0.03)
    parser.add_argument("--max-feedback-age", type=float, default=1.0)
    args = parser.parse_args()
    gate = Validation(args)
    lock = args.runtime.resolve() / "validation.lock"
    owned_lock = False
    try:
        with lock.open("x", encoding="utf-8") as handle:
            json.dump({"pid": os.getpid(), "run_id": gate.run_id, "created_at": time.time()}, handle)
        owned_lock = True
        gate.run()
    except Exception as exc:
        gate.summary["error"] = str(exc)
        gate.summary["traceback"] = traceback.format_exc()
        print(f"FAIL: {exc}", flush=True)
    finally:
        gate.finish()
        if owned_lock:
            lock.unlink()
    return 0 if gate.summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
