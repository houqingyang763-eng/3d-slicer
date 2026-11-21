#!/usr/bin/env python3
"""Compare three MoveIt plans from one frozen UR12e mock start pose."""
import argparse
import fcntl
import json
import math
import os
import sys
import time
import traceback

import rclpy
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState

from scene_tool import PROJECT, SceneTool, atomic_json, pose_json
from ur12e_slicer_bridge.planning import needle_plan_request
from ur12e_slicer_bridge.protocol import shifted_pose
from validate_ros_runtime import Validation, joint_delta, pose_error

sys.path.insert(0, str(PROJECT / "bootstrap"))
from linux_lifecycle import backend_status, slicer_status


PREFIX = "ur12e_keepout_"
NEEDLE = "slicer_simplified_needle"
RADIUS_M = 0.035
GOAL_OFFSET_M = (0.0, 0.20, 0.0)
JOINT_STEP_RAD = 0.01
STATIONARY_RAD = 1e-5
PARAMETERS = (
    "feedback_source", "tip_offset_m", "planning_group", "pipeline_id", "planner_id",
    "planning_time_s", "position_tolerance_m", "orientation_tolerance_rad",
    "velocity_scaling", "acceleration_scaling",
)
NO_PLAN_CODES = {
    MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.TIMED_OUT, MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.GOAL_IN_COLLISION, MoveItErrorCodes.GOAL_STATE_INVALID,
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED, MoveItErrorCodes.NO_IK_SOLUTION,
    # The Jazzy plan service can return generic FAILURE with the rejected candidate still present.
    MoveItErrorCodes.FAILURE,
}
CODE_NAMES = {getattr(MoveItErrorCodes, name): name for name in dir(MoveItErrorCodes)
              if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)}


def joint_samples(trajectory):
    """Include every waypoint, with bounded joint increments between waypoints."""
    points = trajectory.points
    yield {"waypoint_index": 0, "fraction": 0.0, "positions_rad": list(points[0].positions)}
    for index, (left, right) in enumerate(zip(points, points[1:]), start=1):
        count = max(1, math.ceil(max(abs(a - b) for a, b in zip(left.positions, right.positions)) / JOINT_STEP_RAD))
        for step in range(1, count):
            fraction = step / count
            yield {"waypoint_index": index, "fraction": fraction,
                   "positions_rad": [a + fraction * (b - a) for a, b in zip(left.positions, right.positions)]}
        yield {"waypoint_index": index, "fraction": 1.0, "positions_rad": list(right.positions)}


class KeepoutExperiment(Validation):
    def __init__(self, args, scene, settings):
        super().__init__(args)
        self.scene = scene
        self.scene.snapshot_path = self.output / "last_scene_readback.json"
        self.settings = settings
        self.sphere_id = PREFIX + self.run_id
        self.plan_client = scene.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.fk_client = scene.node.create_client(GetPositionFK, "/compute_fk")
        self.joint_count = 0
        self.max_joint_change = 0.0
        self.scene_reference = None
        self.summary.update(mode="ur12e_mock_keepout_plan_only", backend=settings["feedback_source"],
                            frame_id="base", joint_sample_step_rad=JOINT_STEP_RAD, scenes={}, paths={},
                            slicer_requested=not args.no_slicer)

    def observe_joints(self, message):
        observed = dict(zip(message.name, message.position))
        if all(name in observed for name in self.start_joints.name):
            delta = max(abs(observed[name] - start) for name, start in
                        zip(self.start_joints.name, self.start_joints.position))
            self.joint_count += 1
            self.max_joint_change = max(self.max_joint_change, delta)

    def record_scene(self, label, scene):
        self.summary["scenes"][label] = scene
        raw = self.scene.last_scene
        other_geometry = ({obj.id: obj for obj in raw.world.collision_objects if obj.id != self.sphere_id},
                          {item.object.id: item for item in raw.robot_state.attached_collision_objects})
        if NEEDLE not in other_geometry[1]:
            raise RuntimeError("The simplified needle is missing from MoveIt's attached collision objects")
        if self.scene_reference is None:
            self.scene_reference = other_geometry
            atomic_json(self.output / "needle_collision_geometry.json", message_to_ordereddict(other_geometry[1][NEEDLE]))
        elif other_geometry != self.scene_reference:
            raise RuntimeError("Other world or attached geometry changed during the comparison")

    def plan(self, label):
        print(f"Planning: {label}", flush=True)
        timeout = float(self.settings["planning_time_s"]) + 15.0
        response = self.scene.call(self.plan_client, self.request, timeout=timeout).motion_plan_response
        atomic_json(self.output / f"{label}_response.json", message_to_ordereddict(response))
        code = response.error_code.val
        trajectory = response.trajectory.joint_trajectory
        self.summary["cases"].append({"name": label, "moveit_error_code": code,
                                      "moveit_error_name": CODE_NAMES.get(code), "message": response.error_code.message,
                                      "planning_time_s": response.planning_time,
                                      "waypoint_count": len(trajectory.points)})
        if code == MoveItErrorCodes.SUCCESS:
            names = list(trajectory.joint_names)
            if (len(trajectory.points) < 2 or set(names) != set(self.start_joints.name) or
                    len(names) != len(self.start_joints.name) or any(
                        len(point.positions) != len(names) or not all(map(math.isfinite, point.positions))
                        for point in trajectory.points)):
                raise RuntimeError(f"{label}: MoveIt did not return a complete joint path")
            first = {"joint_names": names, "joint_positions_rad": list(trajectory.points[0].positions)}
            start_delta = joint_delta(self.initial, first)
            self.summary["cases"][-1]["first_waypoint_max_start_delta_rad"] = start_delta
            if start_delta >= STATIONARY_RAD:
                raise RuntimeError(f"{label}: trajectory starts at a different joint pose")
        return code, trajectory

    def fk(self, joints, positions):
        state = RobotState(is_diff=True, joint_state=JointState(name=list(joints), position=list(positions)))
        request = GetPositionFK.Request(robot_state=state, fk_link_names=["tool0"])
        request.header.frame_id = "base"
        result = self.scene.call(self.fk_client, request)
        if (result.error_code.val != MoveItErrorCodes.SUCCESS or len(result.pose_stamped) != 1 or
                result.pose_stamped[0].header.frame_id != "base"):
            raise RuntimeError("MoveIt FK did not return the requested tool0 pose in base")
        pose = pose_json(result.pose_stamped[0].pose)
        pose["position_m"] = shifted_pose(pose["position_m"], pose["orientation_xyzw"], self.settings["tip_offset_m"])
        return pose

    def check_samples(self, label, names, samples):
        rows = []
        for sample in samples:
            validity = self.scene.validity(names, sample["positions_rad"], group_name="")
            rows.append({**sample, **validity})
        path = self.output / f"{label}_samples.json"
        atomic_json(path, {"joint_names": list(names), "samples": rows})
        contacts = sorted({tuple(sorted((contact["body_1"], contact["body_2"])))
                           for row in rows for contact in row["contacts"]})
        self.summary["paths"][label] = {"file": path.name, "sample_count": len(rows),
                                         "invalid_sample_count": sum(not row["valid"] for row in rows),
                                         "contact_pairs": contacts}
        return rows

    def check_plan_path(self, label, trajectory):
        names = trajectory.joint_names
        samples = list(joint_samples(trajectory))
        for sample in samples:
            sample["tip_pose_base"] = self.fk(names, sample["positions_rad"])
        rows = self.check_samples(label, names, samples)
        error, angle = pose_error(rows[-1]["tip_pose_base"], self.target)
        points = [row["tip_pose_base"]["position_m"] for row in rows]
        self.summary["paths"][label].update(
            tip_path_length_m=sum(math.dist(a, b) for a, b in zip(points, points[1:])),
            goal_position_error_m=error, goal_orientation_error_rad=angle,
        )
        if error > self.args.position_tolerance or angle > self.args.orientation_tolerance:
            raise RuntimeError(f"{label}: sampled plan endpoint misses the requested pose budget")
        return samples, rows, points

    def compare(self):
        clear = self.scene.snapshot("display_only")
        self.record_scene("display_only", clear)
        if clear["objects"]:
            raise RuntimeError("A previous keep-out sphere is still present. Restart the task mock graph before rerunning.")
        code, baseline = self.plan("display_only")
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Baseline request did not plan successfully (MoveIt {code})")
        samples, clear_rows, points = self.check_plan_path("baseline_clear", baseline)
        if any(not row["valid"] for row in clear_rows):
            raise RuntimeError("Baseline has invalid whole-robot samples before adding a sphere")

        # Choose the sample closest to half the traced needle-tip arc length.
        distances = [0.0]
        for a, b in zip(points, points[1:]):
            distances.append(distances[-1] + math.dist(a, b))
        midpoint = min(range(len(points)), key=lambda index: abs(distances[index] - distances[-1] / 2))
        center = points[midpoint]
        tip_gaps = [math.dist(tip, center) - RADIUS_M for tip in (points[0], points[-1], self.target["position_m"])]
        self.summary["midpoint_sphere"] = {"id": self.sphere_id, "center_m": center, "radius_m": RADIUS_M,
                                           "baseline_sample_index": midpoint, "tip_endpoint_gaps_m": tip_gaps}
        if min(tip_gaps) <= 0.0 or self.args.position_tolerance >= RADIUS_M:
            raise RuntimeError("The default mock start does not give sufficient room for the sphere and target tolerance")

        try:
            self.record_scene("midpath", self.scene.add_sphere(self.sphere_id, center, RADIUS_M))
            blocked = self.check_samples("baseline_with_sphere", baseline.joint_names, samples)
            if not blocked[0]["valid"] or not blocked[-1]["valid"]:
                raise RuntimeError("Midpath sphere collides with the full robot at a control endpoint")
            named_contacts = self.summary["paths"]["baseline_with_sphere"]["contact_pairs"]
            if not any(self.sphere_id in pair for pair in named_contacts):
                raise RuntimeError("MoveIt did not report a sphere contact on the saved baseline path")

            code, replan = self.plan("midpath_sphere")
            if code == MoveItErrorCodes.SUCCESS:
                _, rows, detour = self.check_plan_path("replanned", replan)
                self.summary["paths"]["replanned"]["minimum_tip_distance_to_sphere_surface_m"] = (
                    min(math.dist(tip, center) for tip in detour) - RADIUS_M)
                if any(not row["valid"] for row in rows):
                    raise RuntimeError("The replanned trajectory contains colliding whole-robot samples")
                self.summary["midpath_outcome"] = "detour_valid_at_samples"
            elif code in NO_PLAN_CODES:
                self.summary["midpath_outcome"] = "no_plan_found"
            else:
                raise RuntimeError(f"Unexpected MoveIt response with midpath sphere (code {code})")

            goal_center = self.target["position_m"]
            self.record_scene("goal_blocked", self.scene.add_sphere(self.sphere_id, goal_center, RADIUS_M))
            endpoints = self.check_samples("goal_sphere_endpoints", baseline.joint_names, [samples[0], samples[-1]])
            if (not endpoints[0]["valid"] or endpoints[-1]["valid"] or
                    not any(self.sphere_id in (c["body_1"], c["body_2"]) for c in endpoints[-1]["contacts"])):
                raise RuntimeError("Goal sphere did not block the previously valid goal while leaving the start valid")
            code, trajectory = self.plan("goal_inside_sphere")
            self.summary["goal_blocked"] = code in NO_PLAN_CODES and not trajectory.points
            if not self.summary["goal_blocked"]:
                raise RuntimeError(f"MoveIt did not return the expected no-plan result for the blocked goal (code {code})")
        finally:
            try:
                self.record_scene("cleanup", self.scene.remove(self.sphere_id))
            except Exception as error:
                self.summary["cleanup_error"] = str(error)
                raise

    def run(self):
        self.initial = self.wait_ready()
        atomic_json(self.output / "initial_state.json", self.initial)
        if self.initial["tool"]["tip_offset_m"] != self.settings["tip_offset_m"]:
            raise RuntimeError("Live needle feedback and bridge parameters disagree")
        self.start_joints = JointState(name=self.initial["joint_names"], position=self.initial["joint_positions_rad"])
        self.scene.node.create_subscription(JointState, "/joint_states", self.observe_joints, qos_profile_sensor_data)
        tip = self.initial["tcp_pose"]
        self.target = {"position_m": [x + d for x, d in zip(tip["position_m"], GOAL_OFFSET_M)],
                       "orientation_xyzw": tip["orientation_xyzw"]}
        self.request = needle_plan_request(self.settings.__getitem__, self.start_joints,
                                           self.target["position_m"], self.target["orientation_xyzw"])
        self.summary["target_pose_base"] = self.target
        planner_params = self.scene.node.create_client(GetParameters, "/move_group/get_parameters")
        segment_fraction = f"{self.settings['pipeline_id']}.{self.settings['planning_group']}.longest_valid_segment_fraction"
        response = self.scene.call(planner_params, GetParameters.Request(names=[segment_fraction]))
        self.summary["ompl_longest_valid_segment_fraction"] = parameter_value_to_python(response.values[0])
        atomic_json(self.output / "shared_plan_request.json", message_to_ordereddict(self.request))
        self.compare()

        last = self.read_state()
        self.max_joint_change = max(self.max_joint_change, joint_delta(self.initial, last))
        events = [json.loads(line) for line in self.capture_events().splitlines() if line.strip()]
        concurrent = [event for event in events if event["event"] in
                      {"bridge_started", "request_accepted", "execution_result"}]
        self.summary["motion_check"] = {"joint_messages": self.joint_count,
                                          "maximum_joint_change_rad": self.max_joint_change,
                                          "unexpected_bridge_events": concurrent}
        if self.joint_count == 0 or self.max_joint_change >= STATIONARY_RAD or concurrent:
            raise RuntimeError("Missing stationary-joint evidence or a different bridge operation ran during the experiment")

        if not self.args.no_slicer:
            atomic_json(self.output / "summary.json", self.summary)
            sys.path.insert(0, str(PROJECT / "slicer"))
            from task_client import send

            relative = self.output.relative_to(PROJECT).as_posix()
            reply = send("import runpy\n"
                         "capture = runpy.run_path(str(PROJECT / 'validation/show_keepout_result.py'))['capture']\n"
                         f"_result = capture(PROJECT / {relative!r})\n", timeout=60)
            self.summary["slicer_captures"] = reply["result"]
        self.summary["passed"] = True


def run(args):
    if not backend_status()["ready"]:
        raise RuntimeError("Start this task's mock backend with start_linux.sh first")
    if not args.no_slicer and not slicer_status()["ready"]:
        raise RuntimeError("Start this task's Linux Slicer, or run with --no-slicer")
    if (args.runtime / "navigation_validation.lock").exists():
        raise RuntimeError("The navigation validator already owns the control window")
    lock_path = args.runtime / "validation.lock"
    with lock_path.open("x", encoding="utf-8") as lock:
        json.dump({"pid": os.getpid(), "scope": "keepout_plan_only", "created_at": time.time()}, lock)
    scene = None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        try:
            scene = SceneTool(prefix=PREFIX)
            client = scene.node.create_client(GetParameters, "/ur12e_slicer_bridge/get_parameters")
            response = scene.call(client, GetParameters.Request(names=list(PARAMETERS)))
            settings = dict(zip(PARAMETERS, map(parameter_value_to_python, response.values)))
            if set(settings) != set(PARAMETERS) or None in settings.values() or settings["feedback_source"] != "ros2_mock_hardware":
                raise RuntimeError("Could not read the live mock bridge planning parameters")
            args.position_tolerance = settings["position_tolerance_m"]
            args.orientation_tolerance = settings["orientation_tolerance_rad"]
            experiment = KeepoutExperiment(args, scene, settings)
            try:
                experiment.run()
                print(f"Midpath result: {experiment.summary['midpath_outcome']}; goal inside sphere rejected.", flush=True)
            except (Exception, KeyboardInterrupt) as error:
                experiment.summary.update(error=str(error), traceback=traceback.format_exc())
                print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
            finally:
                experiment.finish()
            return 0 if experiment.summary["passed"] else 1
        finally:
            if scene is not None:
                scene.close()
            rclpy.shutdown()
    finally:
        lock_path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-slicer", action="store_true", help="Record ROS evidence without the Slicer comparison images")
    parser.set_defaults(runtime=PROJECT / "runtime", output=PROJECT / "output/keepout_validation",
                        ready_timeout=15.0, max_feedback_age=1.0)
    args = parser.parse_args()
    args.runtime.mkdir(parents=True, exist_ok=True)
    try:
        with (args.runtime / "linux_lifecycle.lock").open("a") as lifecycle:
            try:
                fcntl.flock(lifecycle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError("Another lifecycle command is using this mock/URSim project") from error
            return run(args)
    except (OSError, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
