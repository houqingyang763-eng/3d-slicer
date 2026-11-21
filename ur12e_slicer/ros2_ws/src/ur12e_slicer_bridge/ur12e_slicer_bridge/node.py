"""Use real MoveIt service/action and observed joint/TF feedback; no ROS emulation."""
from concurrent.futures import ThreadPoolExecutor
import errno
import json
import math
from pathlib import Path
import signal
from threading import Event
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import AttachedCollisionObject, MoveItErrorCodes, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionFK
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from .planning import needle_plan_request, ros_pose
from .protocol import Target, atomic_json, pose_error, shifted_pose


JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
         "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"]
FEEDBACK_SOURCES = {
    "ros2_mock_hardware": "mock hardware",
    "ros2_ursim": "URSim virtual controller",
}


def validate_feedback_source(value):
    if not isinstance(value, str) or value not in FEEDBACK_SOURCES:
        raise ValueError("feedback_source must be ros2_mock_hardware or ros2_ursim; physical hardware is not enabled")
    return value


def json_pose(value):
    return {"position_m": [value.position.x, value.position.y, value.position.z],
            "orientation_xyzw": [value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w]}


class FileBridge(Node):
    def __init__(self):
        super().__init__("ur12e_slicer_bridge")
        defaults = {
            "runtime_dir": "", "base_frame": "base", "tool_frame": "tool0",
            "head_assets_dir": "", "head_planning_strategy": "geometry",
            "feedback_source": "ros2_mock_hardware",
            "tip_offset_m": [0.0, 0.0, 0.12], "planning_group": "ur_manipulator",
            "pipeline_id": "ompl", "planner_id": "",
            "controller_name": "joint_trajectory_controller", "planning_time_s": 5.0,
            "position_tolerance_m": 0.002, "orientation_tolerance_rad": 0.03,
            "velocity_scaling": 0.1, "acceleration_scaling": 0.1, "feedback_hz": 10.0,
            "max_feedback_age_s": 1.0, "command_max_age_s": 60.0, "execution_timeout_s": 120.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda name: self.get_parameter(name).value
        self.feedback_source = validate_feedback_source(self.p("feedback_source"))
        self.backend_description = FEEDBACK_SOURCES[self.feedback_source]
        if not self.p("runtime_dir"):
            raise ValueError("runtime_dir must be the absolute shared ur12e_slicer/runtime directory")
        if self.p("base_frame") != "base" or self.p("tool_frame") != "tool0":
            raise ValueError("Protocol v1 requires base_frame=base and tool_frame=tool0")
        offset = self.p("tip_offset_m")
        if len(offset) != 3 or abs(offset[0]) > 1e-12 or abs(offset[1]) > 1e-12 or not 0 < offset[2] <= 0.3:
            raise ValueError("The simplified needle must extend 0..0.3 m along tool0 local +Z")
        self.runtime = Path(self.p("runtime_dir")).expanduser().resolve()
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.target_file = self.runtime / "target.json"
        self.state_file = self.runtime / "state.json"
        self.events_file = self.runtime / "ros_events.jsonl"
        self.seen = set()
        # Restart must not replay a target left by an earlier process/session.
        self.file_fingerprint = self.fingerprint()
        if self.target_file.exists():
            try:
                self.seen.add(json.loads(self.target_file.read_text(encoding="utf-8-sig")).get("request_id"))
            except (OSError, ValueError):
                pass
        self.last_joint_msg = None
        self.joint_received_at = 0.0
        self.joints = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.scene_pending = False
        self.scene_ready = False
        self.scene_attempt = 0.0
        self.scene_token = 0
        self.active = None
        self.last_target = None
        self.cached_target = None
        self.cached_trajectory = None
        self.goal_handle = None
        self.token = 0
        self.operation_started = 0.0
        self.stage_started = 0.0
        self.state = {
            "schema_version": 1, "source": self.feedback_source, "request_id": None,
            "status": "waiting_for_robot", "message": "Waiting for observed joint states, TF and MoveIt. Submit a fresh command after startup.",
            "plan": {"status": "none", "request_id": None, "point_count": 0, "tcp_path_m": []},
            "execution": {"status": "none"},
            "tool": {"parent_frame": "tool0", "tip_offset_m": list(offset)},
        }
        self.head = None
        self.head_pending = None
        self.head_worker = None
        if self.p("head_assets_dir"):
            if self.feedback_source != "ros2_mock_hardware":
                raise ValueError("Static-head planning requires the fixed local mock backend")
            from .head_planning import HeadPlanner
            from .anatomy_scene import HeadScene
            self.head = HeadScene(self, self.p("head_assets_dir"))
            self.head_solver = HeadPlanner(self)
            self.head_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="head_planning")
            self.head_result_ready = self.create_guard_condition(self.finish_head_plan)
            self.state["anatomy"] = self.head.state
            self.create_timer(.02, self.poll_head_command)
        self.record("bridge_started", {"runtime_dir": str(self.runtime), "source": self.feedback_source,
                    "controller_name": self.p("controller_name"), "startup_target_ignored": self.target_file.exists()})
        self.create_timer(1.0 / float(self.p("feedback_hz")), self.tick)
        self.get_logger().info(f"ROS {self.backend_description} bridge using {self.runtime}; target is needle tip, +Z {offset[2]} m from tool0")

    def fingerprint(self):
        try:
            stat = self.target_file.stat()
            return (stat.st_mtime_ns, stat.st_size)
        except FileNotFoundError:
            return None

    def record(self, event, details):
        payload = {"timestamp": time.time(), "event": event, **details}
        try:
            with self.events_file.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n")
        except OSError as exc:
            self.get_logger().error(f"Cannot append ROS evidence log: {exc}")

    def on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.last_joint_msg = message
            self.joints = {name: values[name] for name in JOINTS}
            self.joint_received_at = time.monotonic()

    def feedback(self):
        transforms = {}
        tf_age = None
        for link in LINKS:
            try:
                transform = self.tf_buffer.lookup_transform("base", link, Time())
            except TransformException:
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            transforms[link] = {"position_m": [t.x, t.y, t.z], "orientation_xyzw": [q.x, q.y, q.z, q.w]}
            if link == "tool0":
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                tf_age = max(0.0, self.get_clock().now().nanoseconds * 1e-9 - stamp)
        tcp = None
        if "tool0" in transforms:
            tool = transforms["tool0"]
            tip = {"position_m": shifted_pose(tool["position_m"], tool["orientation_xyzw"], self.p("tip_offset_m")),
                   "orientation_xyzw": tool["orientation_xyzw"]}
            transforms["needle_tip"] = tip
            tcp = {"frame_id": "base", **tip}
        age = time.monotonic() - self.joint_received_at if self.last_joint_msg else None
        fresh = (age is not None and age < self.p("max_feedback_age_s") and
                 tf_age is not None and tf_age < self.p("max_feedback_age_s") and tcp is not None)
        return transforms, tcp, age, tf_age, fresh

    def tick(self):
        try:
            transforms, tcp, joint_age, tf_age, fresh = self.feedback()
            self.ensure_needle_scene()
            if self.head:
                if self.head.poll_pose():
                    self.invalidate_head_plan()
                self.head.update(transforms)
            else:
                self.read_target(fresh)
            head_ready = self.head_ready()
            self.start_waiting_plan(transforms, fresh, head_ready)
            if self.active and self.state["status"] == "planning" and time.monotonic() - self.stage_started > self.p("planning_time_s") + 15:
                self.fail("Planning service timed out")
            if self.active and self.state["status"] == "executing":
                if time.monotonic() - self.stage_started > self.p("execution_timeout_s"):
                    self.fail("Trajectory execution timed out", cancel=True)
                elif not fresh:
                    self.fail("Observed robot feedback became stale during execution", cancel=True)
            ready = fresh and self.scene_ready and self.plan_client.service_is_ready() and self.execute_client.server_is_ready()
            ready = ready and head_ready
            if self.active is None and self.state["request_id"] is None:
                self.state["status"] = "idle" if ready else "waiting_for_robot"
                self.state["message"] = "Ready for a fresh target" if ready else "Waiting for joint states, TF, needle scene and MoveIt"
            distance = angle = None
            if fresh and tcp and self.last_target:
                distance, angle = pose_error(tcp["position_m"], tcp["orientation_xyzw"],
                                              self.last_target.position_m, self.last_target.orientation_xyzw)
            self.state.update({"timestamp": time.time(), "ready": bool(ready),
                               "joint_names": list(self.joints), "joint_positions_rad": list(self.joints.values()),
                               "joint_state_age_s": joint_age, "tf_age_s": tf_age,
                               "link_transforms": transforms, "tcp_pose": tcp,
                               "target_pose": self.last_target.pose() if self.last_target else None,
                               "position_error_m": distance, "orientation_error_rad": angle,
                               "feedback_valid": bool(fresh), "needle_collision_ready": self.scene_ready})
            atomic_json(self.state_file, self.state)
        except (OSError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f"Bridge tick error: {exc}")

    def head_ready(self):
        return self.head is None or (self.head.ready and self.head_solver.validity_client.service_is_ready())

    def start_waiting_plan(self, transforms, fresh, head_ready):
        if self.active and self.state["status"] == "waiting_for_robot":
            if fresh and self.scene_ready and head_ready and self.plan_client.service_is_ready():
                self.start_plan(transforms)
            elif time.monotonic() - self.operation_started > 20:
                self.fail("No fresh robot feedback/needle scene/planning service within 20 s")

    def poll_head_command(self):
        try:
            fingerprint = self.fingerprint()
            if fingerprint is None or fingerprint == self.file_fingerprint:
                return
            if self.head.poll_pose():
                self.invalidate_head_plan()
            transforms, _, _, _, fresh = self.feedback()
            self.read_target(fresh, fingerprint)
            self.start_waiting_plan(transforms, fresh, self.head_ready())
        except (OSError, ValueError, RuntimeError) as exc:
            self.get_logger().error(f"Head command poll error: {exc}")

    def ensure_needle_scene(self):
        if self.scene_pending and time.monotonic() - self.scene_attempt > 10:
            self.scene_pending = False
            self.scene_token += 1
            self.state["needle_collision_message"] = "ApplyPlanningScene timed out; retrying"
            self.get_logger().error(self.state["needle_collision_message"])
        if self.scene_ready or self.scene_pending or not self.scene_client.service_is_ready():
            return
        if time.monotonic() - self.scene_attempt < 5:
            return
        self.scene_attempt = time.monotonic()
        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.touch_links = ["tool0", "wrist_3_link"]
        attached.object.header.frame_id = "tool0"
        attached.object.id = "slicer_simplified_needle"
        attached.object.operation = attached.object.ADD
        if hasattr(attached.object, "pose"):
            attached.object.pose.orientation.w = 1.0
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [float(self.p("tip_offset_m")[2]), 0.003]
        attached.object.primitives = [cylinder]
        attached.object.primitive_poses = [ros_pose([0.0, 0.0, cylinder.dimensions[0] / 2], [0.0, 0.0, 0.0, 1.0])]
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [attached]
        self.scene_pending = True
        self.scene_token += 1
        token = self.scene_token
        future = self.scene_client.call_async(request)
        future.add_done_callback(lambda f: self.scene_done(f, token))

    def scene_done(self, future, token):
        if token != self.scene_token:
            return
        self.scene_pending = False
        try:
            self.scene_ready = bool(future.result().success)
            self.state["needle_collision_message"] = "Simplified needle attached" if self.scene_ready else "MoveIt rejected simplified needle"
            if not self.scene_ready:
                self.get_logger().error("MoveIt rejected the simplified needle collision object")
        except Exception as exc:
            self.get_logger().error(f"Cannot add simplified needle: {exc}")

    def read_target(self, fresh, fingerprint=None):
        fingerprint = self.fingerprint() if fingerprint is None else fingerprint
        if fingerprint is None or fingerprint == self.file_fingerprint:
            return
        self.file_fingerprint = fingerprint
        data = {}
        try:
            try:
                data = json.loads(self.target_file.read_text(encoding="utf-8-sig"))
            except OSError as exc:
                if exc.errno not in (errno.ENOENT, errno.ENODATA):
                    raise
                # A DrvFS rename can briefly deny access during atomic replacement.
                self.file_fingerprint = None
                return
            target = Target.from_json(data, time.time(), self.p("command_max_age_s"))
            if target.request_id in self.seen:
                return
            self.seen.add(target.request_id)
            if self.active is not None:
                self.state["last_rejected_request_id"] = target.request_id
                self.record("request_rejected", {"request_id": target.request_id, "reason": "busy"})
                self.get_logger().warning(f"Rejected {target.request_id}: another command is active")
                return
            if self.head:
                if data.get("head_revision") != self.head.revision or not self.head.ready:
                    raise ValueError("Head scene revision is stale or unconfirmed; replan against the current head")
                if target.command != "plan":
                    raise ValueError("Static-head mock mode accepts plan commands only")
            self.active = self.last_target = target
            self.token += 1
            self.operation_started = time.monotonic()
            self.state.update({"request_id": target.request_id, "status": "waiting_for_robot", "message": "Waiting for fresh feedback and MoveIt"})
            self.state["execution"] = {"status": "none"}
            self.record("request_accepted", {"request_id": target.request_id, "command": target.command, "target_pose": target.pose()})
            if target.command == "execute":
                if not fresh:
                    self.fail("Fresh joint/TF feedback is required before executing a cached plan")
                else:
                    self.start_execute()
            else:
                self.cached_target = self.cached_trajectory = None
                self.state["plan"] = {"status": "waiting", "request_id": target.request_id, "point_count": 0, "tcp_path_m": []}
        except (OSError, ValueError, TypeError) as exc:
            request_id = data.get("request_id") if isinstance(data, dict) else None
            if self.active:
                self.state["last_rejected_request_id"] = request_id
            else:
                self.state.update({"request_id": request_id, "status": "error", "message": str(exc)})
            self.record("request_rejected", {"request_id": request_id, "reason": str(exc)})

    def start_plan(self, transforms):
        target = self.active
        if self.head:
            self.start_head_plan(target, transforms["tool0"]["orientation_xyzw"])
            return
        request = needle_plan_request(self.p, self.last_joint_msg, target.position_m, target.orientation_xyzw)
        self.state["status"] = "planning"
        self.state["message"] = "MoveIt planning service is computing a collision-checked trajectory"
        self.state["plan"]["status"] = "planning"
        self.stage_started = time.monotonic()
        token = self.token
        future = self.plan_client.call_async(request)
        future.add_done_callback(lambda f: self.plan_done(f, token, target))

    def plan_done(self, future, token, target):
        if token != self.token or self.active is None:
            return
        try:
            response = future.result().motion_plan_response
            code = response.error_code.val
            points = response.trajectory.joint_trajectory.points
            if code != MoveItErrorCodes.SUCCESS or not points:
                self.state["plan"].update({"status": "failed", "moveit_error_code": code})
                self.fail(f"MoveIt planning failed: error_code={code}, trajectory_points={len(points)}")
                return
            self.accept_plan(response, target)
        except Exception as exc:
            self.fail(f"Planning callback failed: {exc}")

    def accept_plan(self, response, target):
        trajectory = response.trajectory.joint_trajectory
        points = trajectory.points
        code = response.error_code.val
        self.cached_target = target
        self.cached_trajectory = response.trajectory
        self.state["plan"].update({"status": "succeeded", "request_id": target.request_id,
                                  "point_count": len(points), "planning_time_s": response.planning_time,
                                  "moveit_error_code": code,
                                  "joint_trajectory": {"joint_names": list(trajectory.joint_names),
                                     "points": [{"positions_rad": list(p.positions), "time_from_start_s": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9} for p in points]}})
        self.record("plan_result", {"request_id": target.request_id, "point_count": len(points),
                                   "moveit_error_code": code, "planning_time_s": response.planning_time})
        if not self.head:
            self.preview_fk(target.request_id, trajectory)
        if target.command == "plan_execute":
            self.start_execute()
        else:
            message = "Needle sweep and whole-arm collision checks passed" if self.head else "Trajectory planned; no execution requested"
            self.state.update(status="planned", message=message)
            self.active = None

    def invalidate_head_plan(self):
        self.cached_target = self.cached_trajectory = None
        self.cancel_head_plan()
        self.state["ready"] = False
        if self.active:
            self.fail("Head pose changed during planning; request a new plan")
        if self.state["plan"].get("request_id"):
            self.state["plan"].update(status="invalidated", point_count=0, tcp_path_m=[], clearance=None,
                                      head_report=None, joint_trajectory=None)
            self.state.update(status="error", message="Head pose changed; previous plan invalidated")
        self.record("anatomy_plan_invalidated", {"revision": self.head.revision})

    def cancel_head_plan(self):
        if self.head_pending is not None:
            future, cancelled, *_ = self.head_pending
            self.head_pending = None
            cancelled.set()
            future.cancel()

    def start_head_plan(self, target, start_orientation):
        import numpy as np
        from .anatomy_geometry import CLEARANCE_MM, JOINT_NAMES
        from .head_planning import HEAD_PLANNER_ID, STRATEGIES, HeadRequest

        strategy = self.p("head_planning_strategy")
        if strategy not in STRATEGIES:
            self.fail(f"Invalid head_planning_strategy: {strategy}")
            return
        verdict = self.head.goal_clearance(target)
        plan = self.state["plan"]
        plan.update(head_revision=self.head.revision, strategy=strategy)
        if verdict["lower_mm"] < CLEARANCE_MM:
            plan["clearance"] = {"passed": False, "reason": "goal_blocked", "margin_mm": CLEARANCE_MM, "closest": verdict}
            self.record("anatomy_goal_rejected", {"request_id": target.request_id, "witness": verdict})
            self.fail(f"Goal rejected by anatomy clearance: {verdict['structure']} / {verdict['part']}")
            return

        request = needle_plan_request(self.p, self.last_joint_msg, target.position_m, target.orientation_xyzw)
        motion = request.motion_plan_request
        motion.planner_id = motion.planner_id or HEAD_PLANNER_ID
        bounds = np.array([mesh["bounds_base_mm"] for mesh in self.head.state["readback"].values()])
        context = HeadRequest(request, strategy, np.array([self.joints[name] for name in JOINT_NAMES]),
                              tuple(start_orientation), np.asarray(target.position_m) * 1000, target.orientation_xyzw,
                              self.p("position_tolerance_m"), self.p("orientation_tolerance_rad"),
                              self.head.kinematics, self.head.clearance, np.linalg.inv(self.head.base_from_head_mm()),
                              np.array([bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)]))
        plan["status"] = "planning"
        self.state.update(status="planning", message="Planning and checking the whole needle and robot against the head")
        self.stage_started = time.monotonic()
        cancelled = Event()
        future = self.head_worker.submit(self.head_solver.solve, context, cancelled)
        self.head_pending = (future, cancelled, self.token, target, self.head.revision, context.start_rad)

        def wake_bridge(_):
            if not cancelled.is_set() and rclpy.ok():
                self.head_result_ready.trigger()

        future.add_done_callback(wake_bridge)

    def finish_head_plan(self):
        from .head_planning import JOINT_NAMES, START_TOLERANCE_RAD

        if not self.head_pending or not self.head_pending[0].done():
            return
        future, _, token, target, revision, start = self.head_pending
        self.head_pending = None
        if self.active is None or token != self.token:
            return
        try:
            if self.head.poll_pose():
                self.invalidate_head_plan()
                return
            if not self.head.ready or revision != self.head.revision:
                self.fail("Head scene is no longer confirmed")
                return
            if (not self.feedback()[-1] or any(abs(self.joints[name] - value) >= START_TOLERANCE_RAD
                                                for name, value in zip(JOINT_NAMES, start))):
                self.fail("Observed robot state changed while planning")
                return
            candidate, search = future.result()
            self.state["plan"]["head_search"] = search
            self.record("anatomy_search_result", {"request_id": target.request_id, "head_revision": revision, **search})
            if candidate is None:
                if search["attempts"]:
                    last = search["attempts"][-1]
                    self.state["plan"]["moveit_error_code"] = last["moveit_error_code"]
                    self.fail(f"No verified head path: {last['reason']}")
                else:
                    self.fail("No verified head path within the planning time")
                return

            verdict = candidate.proof
            filename = f"head_plan_{uuid.uuid4().hex}.json"
            verdict.update(request_id=target.request_id, head_revision=revision,
                           asset_sha256=self.head.state["asset_sha256"], whole_arm=candidate.whole_arm,
                           metrics=candidate.metrics, route=candidate.route)
            atomic_json(self.runtime / filename, verdict)
            if self.head.poll_pose():
                self.invalidate_head_plan()
                return
            summary = {key: verdict[key] for key in ("passed", "reason", "margin_mm", "certified_minimum_lower_mm",
                                                      "sample_count", "interval_count")}
            summary["closest"] = {key: value for key, value in verdict["closest"].items() if key != "links_base_mm"}
            self.state["plan"].update(clearance=summary, head_report=filename, whole_arm=candidate.whole_arm,
                                      metrics=candidate.metrics)
            self.record("anatomy_sweep_result", {"request_id": target.request_id, "head_revision": revision, **summary})
            self.state["plan"]["tcp_path_m"] = [[value / 1000 for value in frame["tip_base_mm"]]
                                                for frame in verdict["frames"]]
            search["operation_wall_s"] = time.monotonic() - self.operation_started
            self.accept_plan(candidate.response, target)
        except Exception as error:
            self.fail(f"Head planning failed: {error}")
        finally:
            try:
                atomic_json(self.state_file, self.state)
            except OSError as error:
                self.get_logger().error(f"Cannot publish head plan result: {error}")

    def start_execute(self):
        target = self.active
        if not self.feedback()[-1]:
            self.fail("Fresh joint/TF feedback is required before trajectory execution")
            return
        if self.cached_trajectory is None or self.cached_target is None:
            self.fail("No successful cached plan; plan first")
            return
        if target.command == "execute" and (target.plan_request_id != self.cached_target.request_id or not target.same_pose(self.cached_target)):
            self.fail("Cached plan_request_id/target pose mismatch; replan")
            return
        trajectory = self.cached_trajectory.joint_trajectory
        if any(name not in self.joints or abs(self.joints[name] - value) > 0.01
               for name, value in zip(trajectory.joint_names, trajectory.points[0].positions)):
            self.fail("Robot joints changed since planning (>0.01 rad); replan")
            return
        if not self.execute_client.server_is_ready():
            self.fail("MoveIt /execute_trajectory action server is unavailable")
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self.cached_trajectory
        # Select the controller actually activated by this backend: ordinary JTC
        # for mock.launch.py, scaled JTC for ursim.launch.py.
        goal.controller_names = [self.p("controller_name")]
        self.state.update({"status": "executing", "message": f"Executing trajectory through MoveIt on {self.backend_description}"})
        self.state["execution"] = {"status": "requesting"}
        self.stage_started = time.monotonic()
        token = self.token
        future = self.execute_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.goal_done(f, token))

    def goal_done(self, future, token):
        try:
            handle = future.result()
            if token != self.token or self.active is None:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.fail("MoveIt rejected ExecuteTrajectory goal")
                return
            self.goal_handle = handle
            self.state["execution"]["status"] = "executing"
            handle.get_result_async().add_done_callback(lambda f: self.execution_done(f, token))
        except Exception as exc:
            self.fail(f"Cannot submit trajectory: {exc}")

    def execution_done(self, future, token):
        if token != self.token or self.active is None:
            return
        try:
            result = future.result()
            code = result.result.error_code.val
            request_id = self.active.request_id
            self.state["execution"].update({"moveit_error_code": code, "action_status": result.status})
            if result.status != GoalStatus.STATUS_SUCCEEDED or code != MoveItErrorCodes.SUCCESS:
                self.fail(f"MoveIt execution failed: action_status={result.status}, error_code={code}")
                return
            self.state["execution"]["status"] = "succeeded"
            self.state.update({"status": "succeeded", "message": f"MoveIt reports successful {self.backend_description} execution; inspect separately measured pose errors"})
            _, tcp, _, _, fresh = self.feedback()
            errors = {}
            if fresh and tcp:
                distance, angle = pose_error(tcp["position_m"], tcp["orientation_xyzw"], self.active.position_m, self.active.orientation_xyzw)
                errors = {"position_error_m": distance, "orientation_error_rad": angle, "tcp_pose": tcp}
            self.record("execution_result", {"request_id": request_id, "moveit_error_code": code, "action_status": result.status, **errors})
            self.active = None
            self.goal_handle = None
            self.cached_trajectory = None  # A trajectory is consumed once.
        except Exception as exc:
            self.fail(f"Execution result failed: {exc}")

    def preview_fk(self, plan_id, trajectory):
        if not self.fk_client.service_is_ready():
            self.state["plan"]["preview_message"] = "/compute_fk unavailable; no synthetic preview drawn"
            return
        points = trajectory.points
        indices = sorted(set(round(i * (len(points) - 1) / min(23, max(1, len(points) - 1))) for i in range(min(24, len(points)))))
        samples = []

        def next_sample(index):
            if self.state["plan"].get("request_id") != plan_id:
                return
            if index >= len(indices):
                self.state["plan"]["tcp_path_m"] = samples
                self.state["plan"]["preview_message"] = "Needle-tip samples computed by MoveIt FK from the actual planned joint trajectory"
                return
            request = GetPositionFK.Request()
            request.header.frame_id = "base"
            request.fk_link_names = ["tool0"]
            request.robot_state = RobotState()
            request.robot_state.is_diff = True
            request.robot_state.joint_state.name = list(trajectory.joint_names)
            request.robot_state.joint_state.position = list(points[indices[index]].positions)
            future = self.fk_client.call_async(request)

            def done(result):
                if self.state["plan"].get("request_id") != plan_id:
                    return
                try:
                    response = result.result()
                    if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
                        self.state["plan"]["preview_message"] = "MoveIt FK failed; preview remains empty"
                        return
                    pose = json_pose(response.pose_stamped[0].pose)
                    samples.append(shifted_pose(pose["position_m"], pose["orientation_xyzw"], self.p("tip_offset_m")))
                    next_sample(index + 1)
                except Exception as exc:
                    self.state["plan"]["preview_message"] = f"MoveIt FK failed: {exc}"

            future.add_done_callback(done)

        next_sample(0)

    def fail(self, message, cancel=False):
        request_id = self.active.request_id if self.active else self.state.get("request_id")
        self.cancel_head_plan()
        if cancel and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.state["status"] == "executing":
            self.state["execution"]["status"] = "failed"
        elif self.state["status"] in ("planning", "waiting_for_robot"):
            self.state["plan"]["status"] = "failed"
        self.state.update({"status": "error", "message": message})
        self.record("request_failed", {"request_id": request_id, "reason": message})
        self.get_logger().error(message)
        self.active = None
        self.cached_trajectory = None
        self.goal_handle = None
        self.token += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FileBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch or a terminal can deliver another SIGINT after shutdown starts.
        # Allow node destruction to finish instead of interrupting its cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if node.head_worker is not None:
                node.cancel_head_plan()
                node.head_worker.shutdown(wait=False, cancel_futures=True)
            if node.goal_handle is not None and rclpy.ok():
                node.goal_handle.cancel_goal_async()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
