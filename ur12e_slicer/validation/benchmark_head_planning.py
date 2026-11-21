#!/usr/bin/env python3
"""Compare the head planner with tuned RRTConnect through the normal mock file bridge."""
import argparse
from collections import Counter
import errno
import fcntl
import json
import os
import random
from statistics import median
import sys
import time
import traceback
import uuid

from moveit_msgs.srv import GetPlannerParams
import numpy as np
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.parameter import Parameter, parameter_value_to_python
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scene_tool import PROJECT, SceneTool, atomic_json
from ur12e_slicer_bridge.anatomy_geometry import ArmKinematics, CLEARANCE_MM, JOINT_NAMES
from ur12e_slicer_bridge.anatomy_scene import DEFAULT_POSITION_M, OBJECT_IDS
from ur12e_slicer_bridge.head_planning import HEAD_PLANNER_ID, STRATEGIES
from validate_keepout import joint_samples
from validate_ros_runtime import Validation, joint_delta

sys.path.insert(0, str(PROJECT / "bootstrap"))
from linux_lifecycle import backend_status


SEED = 3412
PARAMETER = "head_planning_strategy"
LOCAL_CASES = (
    ("nominal", [0, 0, 0], 0, [0, 0, 0], 0),
    ("head_shift", [0, 0, -12], 10, [0, 0, 0], 0),
    ("tilted_goal", [0, 0, 0], 0, [30, -12, 16], 12),
    ("head_high", [30, 12, 14], -18, [24, 25, 12], -14),
    ("head_left", [-20, -10, -5], 20, [-17, 15, 25], 9),
    ("straight_clear", [320, 0, -35], 0, [0, 0, 0], 0),
)


def percentiles(values):
    if not values:
        return None
    return {"median": median(values), "p90": float(np.percentile(values, 90))}


class HeadBenchmark(Validation):
    def __init__(self, args, scene):
        super().__init__(args)
        self.scene = scene
        self.scene.snapshot_path = self.output / "active_scene.json"
        self.bridge = "/ur12e_slicer_bridge"
        self.set_client = scene.node.create_client(SetParameters, self.bridge + "/set_parameters")
        self.plan_parameters = scene.node.create_client(GetPlannerParams, "/get_planner_params")
        self.checked = set()
        self.summary.update(mode="mock_head_planner_comparison", repeats=args.repeats, seed=SEED, scenes={},
                            transient_state_file_misses=0,
                            maximum_observed_joint_change_rad=0.0)

    def get_parameter(self, node, name):
        client = self.scene.node.create_client(GetParameters, node + "/get_parameters")
        try:
            reply = self.scene.call(client, GetParameters.Request(names=[name]))
            return parameter_value_to_python(reply.values[0])
        finally:
            self.scene.node.destroy_client(client)

    def set_strategy(self, strategy):
        request = SetParameters.Request(parameters=[Parameter(PARAMETER, value=strategy).to_parameter_msg()])
        reply = self.scene.call(self.set_client, request)
        assert reply.results[0].successful, reply.results[0].reason

    def observe(self):
        deadline = time.monotonic() + .04
        while True:
            try:
                state = self.read_state()
                break
            except OSError as error:
                if error.errno not in (errno.ENOENT, errno.ENODATA) or time.monotonic() >= deadline:
                    raise
                self.summary["transient_state_file_misses"] += 1
                time.sleep(.005)
        delta = joint_delta(self.initial, state)
        self.summary["maximum_observed_joint_change_rad"] = max(delta, self.summary["maximum_observed_joint_change_rad"])
        assert delta < 1e-5 and state["execution"]["status"] == "none", "Mock moved during plan-only evaluation"
        return state

    def set_head(self, name, position, orientation):
        request = {"session": self.initial["anatomy"]["session"], "revision": uuid.uuid4().hex,
                   "position_m": list(position), "orientation_xyzw": list(orientation)}
        atomic_json(self.runtime / "head_pose.json", request)
        deadline = time.monotonic() + self.args.ready_timeout
        while time.monotonic() < deadline:
            state = self.observe()
            head = state["anatomy"]
            if state["ready"] and head["ready"] and head["confirmed_revision"] == head["revision"] == request["revision"]:
                assert head["position_m"] == request["position_m"] and head["orientation_xyzw"] == request["orientation_xyzw"]
                scene = self.scene.snapshot(name)
                assert set(OBJECT_IDS.values()) <= set(scene["world_object_ids"])
                assert "slicer_simplified_needle" in scene["attached_object_ids"]
                self.summary["scenes"][name] = {"scene": scene, "head": head}
                return head["revision"]
            time.sleep(.02)
        raise TimeoutError(f"Scene {name} never confirmed")

    def verify(self, name, strategy, request, plan):
        report = json.loads((self.runtime / plan["head_report"]).read_text(encoding="utf-8"))
        assert report["request_id"] == request["request_id"] and report["head_revision"] == request["head_revision"]
        assert report["passed"] and report["certified_minimum_lower_mm"] >= CLEARANCE_MM
        assert report["whole_arm"]["passed"] and report["whole_arm"]["joint_step_rad"] <= .01
        points = plan["joint_trajectory"]
        joint_order = [points["joint_names"].index(joint) for joint in JOINT_NAMES]
        trajectory = JointTrajectory(joint_names=points["joint_names"], points=[
            JointTrajectoryPoint(positions=point["positions_rad"]) for point in points["points"]])
        tips = []
        key = (name, strategy)
        recheck = key not in self.checked
        for sample in joint_samples(trajectory):
            positions = sample["positions_rad"]
            if recheck:
                assert self.scene.validity(trajectory.joint_names, positions, group_name="")["valid"], key
            pose = self.arm.link_transform("tool0", [positions[index] for index in joint_order])
            tips.append(pose[:3, 3] + self.needle_mm * pose[:3, 2])
        self.checked.add(key)
        goal = np.asarray(request["position_m"]) * 1000
        assert np.linalg.norm(tips[-1] - goal) <= self.args.position_tolerance * 1000 + 1e-6
        increments = np.diff(np.array(tips), axis=0)
        direction = goal - tips[0]
        norm = np.linalg.norm(direction)
        backward = float(np.maximum(0, -(increments @ (direction / norm))).sum()) if norm > 1e-6 else 0.0
        return {"tip_length_mm": float(np.linalg.norm(increments, axis=1).sum()), "backtrack_mm": backward,
                "clearance_lower_mm": report["certified_minimum_lower_mm"], "whole_arm_samples": report["whole_arm"]["sample_count"],
                "independent_state_recheck": recheck}

    def trial(self, name, revision, goal, orientation, strategy):
        self.set_strategy(strategy)
        request = {"schema_version": 1, "frame_id": "base", "command": "plan", "head_revision": revision,
                   "request_id": uuid.uuid4().hex, "timestamp": time.time(), "position_m": list(goal),
                   "orientation_xyzw": list(orientation)}
        started = time.monotonic()
        atomic_json(self.runtime / "target.json", request)
        while time.monotonic() - started < self.args.step_timeout:
            state = self.observe()
            assert state["anatomy"]["revision"] == revision
            if state["request_id"] == request["request_id"] and state["status"] in ("planned", "error"):
                elapsed = time.monotonic() - started
                break
            assert state.get("last_rejected_request_id") != request["request_id"], "Planning request was rejected as busy"
            time.sleep(.004)
        else:
            raise TimeoutError(f"No terminal result for {name} {strategy}")

        plan = state["plan"]
        assert plan["request_id"] == request["request_id"] and plan["strategy"] == strategy
        if strategy == "ompl":
            assert plan["head_search"]["attempts"][0]["planner_id"] == HEAD_PLANNER_ID
        accepted = state["status"] == "planned"
        metrics = self.verify(name, strategy, request, plan) if accepted else None
        return {"scene": name, "strategy": strategy, "request": request, "accepted": accepted, "message": state["message"],
                "file_roundtrip_s": elapsed, "search": plan.get("head_search"), "metrics": metrics, "plan": plan}

    def collect(self, rows):
        groups = {}
        for group, names in (("detour", [row[0] for row in LOCAL_CASES[:-1]]), ("straight", ["straight_clear"])):
            groups[group] = {}
            for strategy in STRATEGIES:
                relevant = [row for row in rows if row["scene"] in names and row["strategy"] == strategy]
                passed = [row for row in relevant if row["accepted"]]
                groups[group][strategy] = {"accepted": len(passed), "total": len(relevant),
                                           "routes": dict(Counter(row["search"]["selected_route"] for row in passed))}
                values = groups[group][strategy]
                for field in ("solve_wall_s", "operation_wall_s"):
                    values[field] = percentiles([row["search"][field] for row in passed])
                values["file_roundtrip_s"] = percentiles([row["file_roundtrip_s"] for row in passed])
                for field in ("tip_length_mm", "backtrack_mm"):
                    values[field] = percentiles([row["metrics"][field] for row in passed])
        for group, values in groups.items():
            fast, baseline = values["geometry"], values["ompl"]
            if fast["operation_wall_s"] and baseline["operation_wall_s"]:
                values["operation_median_speedup"] = baseline["operation_wall_s"]["median"] / fast["operation_wall_s"]["median"]
        self.summary["performance"] = groups
        self.summary["independently_rechecked_scene_strategy_pairs"] = sorted(self.checked)

    def run(self):
        self.initial = self.wait_ready()
        assert self.initial["anatomy"]["ready"]
        assert self.initial["anatomy"]["position_m"] == DEFAULT_POSITION_M, "Restore the default head position first"
        assert self.initial["anatomy"]["orientation_xyzw"] == [0, 0, 0, 1], "Restore the default head orientation first"
        origin = np.asarray(self.initial["anatomy"]["position_m"])
        original_quat = self.initial["anatomy"]["orientation_xyzw"]
        tip = np.asarray(self.initial["tcp_pose"]["position_m"])
        orientation = Rotation.from_quat(self.initial["tcp_pose"]["orientation_xyzw"])
        self.needle_mm = self.initial["tool"]["tip_offset_m"][2] * 1000
        self.arm = ArmKinematics(self.get_parameter("/robot_state_publisher", "robot_description"), self.needle_mm)
        config = self.scene.call(self.plan_parameters, GetPlannerParams.Request(
            pipeline_id="ompl", group="ur_manipulator", planner_config=HEAD_PLANNER_ID)).params
        profile = dict(zip(config.keys, config.values, strict=True))
        assert profile["type"] == "geometric::RRTConnect" and float(profile["range"]) == .70
        self.summary["generic_profile"] = profile
        original_strategy = self.get_parameter(self.bridge, PARAMETER)
        randomizer = random.Random(SEED)
        rows = []
        try:
            with (self.output / "trials.jsonl").open("x", encoding="utf-8") as log:
                def measure(name, revision, goal, quat, strategy):
                    # Sample several phases of both the command and feedback timers.
                    time.sleep(randomizer.uniform(0, .1))
                    row = self.trial(name, revision, goal, quat, strategy)
                    log.write(json.dumps(row, allow_nan=False) + "\n")
                    log.flush()
                    rows.append(row)
                    return row

                for name, head_shift, yaw, goal_shift, tilt in LOCAL_CASES:
                    q_head = Rotation.from_euler("z", yaw, degrees=True).as_quat()
                    revision = self.set_head(name, origin + np.array(head_shift) / 1000, q_head)
                    goal = tip + [0, .35, 0] + np.array(goal_shift) / 1000
                    quat = (orientation * Rotation.from_euler("x", tilt, degrees=True)).as_quat()
                    for _ in range(self.args.repeats):
                        for strategy in randomizer.sample(STRATEGIES, len(STRATEGIES)):
                            measure(name, revision, goal, quat, strategy)
                    print(f"Completed {name}: {self.args.repeats} per strategy", flush=True)

                revision = self.set_head("controls", origin, original_quat)
                goal = tip + [-.02, -.12, .08]
                quat = (orientation * Rotation.from_euler("x", 52, degrees=True)).as_quat()
                fallback = measure("large_tilt_fallback", revision, goal, quat, "geometry")
                assert fallback["accepted"] and fallback["search"]["selected_route"] == "ompl", fallback["message"]
                for label, relative in (("tip_inside_brain", [0, 0, .034]), ("shaft_inside_brain", [0, 0, -.110])):
                    rejected = measure(label, revision, origin + Rotation.from_quat(original_quat).apply(relative),
                                       orientation.as_quat(), "geometry")
                    witness = rejected["plan"]["clearance"]
                    assert not rejected["accepted"] and witness["reason"] == "goal_blocked" and rejected["search"] is None

            self.collect(rows)
            assert all(any(row["accepted"] and row["strategy"] == "geometry" and row["scene"] == name
                           for row in rows) for name, *_ in LOCAL_CASES)
            events = [json.loads(line) for line in self.capture_events().splitlines()]
            assert not any(event.get("command") in ("execute", "plan_execute") or event["event"] == "execution_result"
                           for event in events)
            self.summary.update(passed=True, evaluations=len(rows), fallback_verified=True,
                                rejected_controls=2, execution_requests=0)
        finally:
            try:
                self.set_head("restored", origin, original_quat)
            finally:
                self.set_strategy(original_strategy)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=15)
    parser.set_defaults(runtime=PROJECT / "runtime", output=PROJECT / "output/head_planning_benchmark", ready_timeout=12.0,
                        step_timeout=25.0, max_feedback_age=1.0, position_tolerance=.002, orientation_tolerance=.03)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    status = backend_status()
    if not status.get("anatomy") or not status["ready"]:
        parser.error("Start the mock backend with start_linux.sh --anatomy")
    with (args.runtime / "linux_lifecycle.lock").open("a") as lifecycle:
        fcntl.flock(lifecycle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert not (args.runtime / "navigation_validation.lock").exists(), "The anatomy validator owns the bridge"
        lock = args.runtime / "validation.lock"
        with lock.open("x") as stream:
            json.dump({"mode": "head_planning_benchmark", "pid": os.getpid()}, stream)
        try:
            rclpy.init(args=[])
            scene = SceneTool()
            try:
                benchmark = HeadBenchmark(args, scene)
                try:
                    benchmark.run()
                except Exception as error:
                    benchmark.summary.update(passed=False, error=str(error), traceback=traceback.format_exc())
                    print(f"FAIL {error}", file=sys.stderr)
                finally:
                    benchmark.finish()
                return 0 if benchmark.summary["passed"] else 1
            finally:
                scene.close()
                rclpy.shutdown()
        finally:
            lock.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
