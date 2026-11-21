"""Plan-only head routing and a common final gate for Cartesian and OMPL paths."""
from dataclasses import dataclass
from itertools import islice
import math
from threading import Event
import time

from moveit_msgs.msg import MotionPlanResponse, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetStateValidity
import numpy as np
from sensor_msgs.msg import JointState

from .anatomy_geometry import ArmKinematics, HeadClearance, JOINT_NAMES, certify_sweep, pose_matrix_mm
from .head_routes import WHOLE_ARM_STEP_RAD, head_routes, joint_samples, path_quality, route_waypoints
from .planning import ros_pose


HEAD_PLANNER_ID = "HeadRRTConnect"
STRATEGIES = ("geometry", "ompl")
CARTESIAN_STEP_M = 0.004
START_TOLERANCE_RAD = 1e-5
VALIDITY_BATCH_SIZE = 8


@dataclass(frozen=True)
class HeadRequest:
    motion: GetMotionPlan.Request
    strategy: str
    start_rad: np.ndarray
    start_orientation: tuple
    goal_mm: np.ndarray
    goal_orientation: tuple
    position_tolerance_m: float
    orientation_tolerance_rad: float
    kinematics: ArmKinematics
    clearance: HeadClearance
    head_from_base: np.ndarray
    bounds_mm: np.ndarray


@dataclass
class Candidate:
    route: str
    response: MotionPlanResponse
    proof: dict
    metrics: dict
    whole_arm: dict


class HeadPlanner:
    def __init__(self, bridge):
        self.motion_client = bridge.plan_client
        self.cartesian_client = bridge.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.validity_client = bridge.create_client(GetStateValidity, "/check_state_validity")

    @staticmethod
    def request_batch(client, requests, deadline, cancelled):
        """The worker waits while the bridge executor handles the ROS responses."""
        wake = Event()
        futures = []
        try:
            for request in requests:
                if cancelled.is_set() or time.monotonic() >= deadline:
                    raise TimeoutError(f"Interrupted head planning service {client.srv_name}")
                future = client.call_async(request)
                futures.append(future)
                future.add_done_callback(lambda _: wake.set())
            while not all(f.done() for f in futures):
                remaining = deadline - time.monotonic()
                if cancelled.is_set() or remaining <= 0:
                    raise TimeoutError(f"Interrupted head planning service {client.srv_name}")
                wake.wait(min(.03, remaining))
                wake.clear()
            return [f.result() for f in futures]
        finally:
            for future in futures:
                if not future.done():
                    client.remove_pending_request(future)
                    future.cancel()

    def cartesian(self, context, tips, start_tip, deadline, cancelled):
        waypoints = route_waypoints(start_tip, context.start_orientation, context.goal_orientation,
                                   tips, context.kinematics.tip_length_mm)
        motion = context.motion.motion_plan_request
        request = GetCartesianPath.Request(start_state=motion.start_state, group_name=motion.group_name,
                                           link_name="tool0", waypoints=[ros_pose(p, q) for p, q in waypoints],
                                           max_step=CARTESIAN_STEP_M, avoid_collisions=True,
                                           max_velocity_scaling_factor=motion.max_velocity_scaling_factor,
                                           max_acceleration_scaling_factor=motion.max_acceleration_scaling_factor)
        request.header.frame_id = "base"
        started = time.monotonic()
        response, = self.request_batch(self.cartesian_client, [request], deadline, cancelled)
        elapsed = time.monotonic() - started
        return (MotionPlanResponse(trajectory=response.solution, error_code=response.error_code, planning_time=elapsed),
                response.fraction)

    def whole_arm(self, positions, deadline, cancelled):
        samples = iter(joint_samples(positions))
        checked = 0
        while batch := list(islice(samples, VALIDITY_BATCH_SIZE)):
            requests = [GetStateValidity.Request(robot_state=RobotState(is_diff=True, joint_state=JointState(
                name=list(JOINT_NAMES), position=point.tolist()))) for point in batch]
            responses = self.request_batch(self.validity_client, requests, deadline, cancelled)
            for point, response in zip(batch, responses, strict=True):
                checked += 1
                if not response.valid:
                    return {"passed": False, "sample_count": checked, "joint_step_rad": WHOLE_ARM_STEP_RAD,
                            "joints_rad": point.tolist(),
                            "contacts": [{"body_1": c.contact_body_1, "body_2": c.contact_body_2,
                                          "depth_m": c.depth} for c in response.contacts]}
        return {"passed": True, "sample_count": checked, "joint_step_rad": WHOLE_ARM_STEP_RAD}

    def verify(self, context, response, attempt, deadline, cancelled):
        trajectory = response.trajectory.joint_trajectory
        code = response.error_code.val
        attempt.update(moveit_error_code=code, point_count=len(trajectory.points))
        if code != MoveItErrorCodes.SUCCESS or not trajectory.points:
            attempt["reason"] = "moveit_rejected"
            return None
        fraction = attempt.get("cartesian_fraction", 1.0)
        if not math.isfinite(fraction) or fraction < 1 - 1e-8:
            attempt["reason"] = "partial_cartesian_path"
            return None
        if len(trajectory.joint_names) != len(JOINT_NAMES) or set(trajectory.joint_names) != set(JOINT_NAMES):
            attempt["reason"] = "missing_ur_joints"
            return None
        if any(len(point.positions) != len(JOINT_NAMES) for point in trajectory.points):
            attempt["reason"] = "invalid_trajectory"
            return None
        order = [trajectory.joint_names.index(name) for name in JOINT_NAMES]
        positions = np.asarray([list(point.positions) for point in trajectory.points])[:, order]
        times = np.asarray([point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                            for point in trajectory.points])
        time_steps = np.diff(times)
        if (not np.isfinite(positions).all() or times[0] < 0 or np.any(time_steps < 0)
                or np.any((time_steps == 0) & np.any(np.diff(positions, axis=0) != 0, axis=1))):
            attempt["reason"] = "invalid_trajectory"
            return None
        start_error = float(np.max(np.abs(positions[0] - context.start_rad)))
        final = context.kinematics.link_transform("tool0", positions[-1])
        tip = final[:3, 3] + context.kinematics.tip_length_mm * final[:3, 2]
        goal_rotation = pose_matrix_mm(context.goal_mm, context.goal_orientation)[:3, :3]
        position_error = float(np.linalg.norm(tip - context.goal_mm))
        angle_error = math.acos(float(np.clip((np.trace(final[:3, :3].T @ goal_rotation) - 1) / 2, -1, 1)))
        attempt.update(start_error_rad=start_error, tip_error_mm=position_error, orientation_error_rad=angle_error)
        if (start_error >= START_TOLERANCE_RAD or position_error > context.position_tolerance_m * 1000 + 1e-6
                or angle_error > context.orientation_tolerance_rad + 1e-7):
            attempt["reason"] = "endpoint_mismatch"
            return None
        if len(positions) == 1:
            positions, times = np.repeat(positions, 2, axis=0), np.repeat(times, 2)
        proof = certify_sweep(positions, times, context.kinematics, context.clearance, context.head_from_base)
        if not proof["passed"]:
            attempt.update(reason=proof["reason"], clearance_witness={key: value for key, value in proof["closest"].items()
                                                                    if key != "links_base_mm"})
            return None
        whole_arm = self.whole_arm(positions, deadline, cancelled)
        attempt["whole_arm"] = whole_arm
        if not whole_arm["passed"]:
            attempt["reason"] = "whole_arm_collision"
            return None
        metrics = path_quality(proof["frames"], context.goal_mm)
        attempt.update(reason="passed", metrics=metrics, clearance_lower_mm=proof["certified_minimum_lower_mm"])
        return Candidate(attempt["route"], response, proof, metrics, whole_arm)

    def solve(self, context, cancelled):
        started = time.monotonic()
        motion = context.motion.motion_plan_request
        budget = motion.allowed_planning_time
        search_until = started + budget
        verify_until = search_until + 2.0
        if not self.validity_client.service_is_ready():
            raise RuntimeError("MoveIt whole-arm validity service is unavailable")
        attempts = []
        best = None
        straight_blocked = False

        def evaluate(route, response, began, **details):
            nonlocal best
            attempt = {"route": route, "service_wall_s": time.monotonic() - began, **details}
            candidate = self.verify(context, response, attempt, verify_until, cancelled)
            attempt["wall_s"] = time.monotonic() - began
            attempts.append(attempt)
            if candidate:
                score = (not candidate.metrics["short_forward_route"], candidate.metrics["score_mm"])
                if best is None or score < (not best.metrics["short_forward_route"], best.metrics["score_mm"]):
                    best = candidate
            return candidate is not None and candidate.metrics["short_forward_route"]

        if context.strategy == "geometry" and self.cartesian_client.service_is_ready():
            pose = context.kinematics.link_transform("tool0", context.start_rad)
            needle = context.kinematics.tip_length_mm
            tip = pose[:3, 3] + needle * pose[:3, 2]
            routes, straight_blocked = head_routes(pose, context.start_orientation, context.goal_mm,
                                                   context.goal_orientation, context.bounds_mm,
                                                   context.head_from_base, context.clearance, needle)
            for label, tips in routes:
                if cancelled.is_set() or time.monotonic() >= search_until:
                    break
                began = time.monotonic()
                response, fraction = self.cartesian(context, tips, tip, verify_until, cancelled)
                if evaluate(label, response, began, cartesian_fraction=float(fraction)):
                    break

        if best is None or not best.metrics["short_forward_route"]:
            retries = 3 if context.strategy == "geometry" else 1
            for _ in range(retries):
                began = time.monotonic()
                remaining = search_until - began
                if cancelled.is_set() or remaining < .02:
                    break
                motion.allowed_planning_time = remaining
                reply, = self.request_batch(self.motion_client, [context.motion], verify_until, cancelled)
                if evaluate("ompl", reply.motion_plan_response, began, planner_id=motion.planner_id):
                    break

        return best, {"strategy": context.strategy, "straight_predicted_blocked": straight_blocked,
                      "attempts": attempts, "solve_wall_s": time.monotonic() - started,
                      "selected_route": best.route if best else None}
