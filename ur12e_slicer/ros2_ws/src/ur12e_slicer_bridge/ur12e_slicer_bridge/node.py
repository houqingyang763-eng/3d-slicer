"""Use real MoveIt service/action and observed joint/TF feedback; no ROS emulation."""
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import AttachedCollisionObject, Constraints, MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionFK
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from .protocol import Target, atomic_json, pose_error, shifted_pose


JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
         "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"]


def ros_pose(position, quaternion):
    value = Pose()
    value.position.x, value.position.y, value.position.z = map(float, position)
    value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w = map(float, quaternion)
    return value


def json_pose(value):
    return {"position_m": [value.position.x, value.position.y, value.position.z],
            "orientation_xyzw": [value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w]}


class FileBridge(Node):
    def __init__(self):
        super().__init__("ur12e_slicer_bridge")
        defaults = {
            "runtime_dir": "", "base_frame": "base", "tool_frame": "tool0",
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
            "schema_version": 1, "source": "ros2_mock_hardware", "request_id": None,
            "status": "waiting_for_robot", "message": "Waiting for observed joint states, TF and MoveIt. Submit a fresh command after startup.",
            "plan": {"status": "none", "request_id": None, "point_count": 0, "tcp_path_m": []},
            "execution": {"status": "none"},
            "tool": {"parent_frame": "tool0", "tip_offset_m": list(offset)},
        }
        self.record("bridge_started", {"runtime_dir": str(self.runtime), "startup_target_ignored": self.target_file.exists()})
        self.create_timer(1.0 / float(self.p("feedback_hz")), self.tick)
        self.get_logger().info(f"ROS mock-hardware bridge using {self.runtime}; target is needle tip, +Z {offset[2]} m from tool0")

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
            self.read_target(fresh)
            if self.active and self.state["status"] == "waiting_for_robot":
                if fresh and self.scene_ready and self.plan_client.service_is_ready():
                    self.start_plan()
                elif time.monotonic() - self.operation_started > 20:
                    self.fail("No fresh robot feedback/needle scene/planning service within 20 s")
            if self.active and self.state["status"] == "planning" and time.monotonic() - self.stage_started > self.p("planning_time_s") + 15:
                self.fail("Planning service timed out")
            if self.active and self.state["status"] == "executing":
                if time.monotonic() - self.stage_started > self.p("execution_timeout_s"):
                    self.fail("Trajectory execution timed out", cancel=True)
                elif not fresh:
                    self.fail("Observed robot feedback became stale during execution", cancel=True)
            ready = fresh and self.scene_ready and self.plan_client.service_is_ready() and self.execute_client.server_is_ready()
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

    def read_target(self, fresh):
        fingerprint = self.fingerprint()
        if fingerprint is None or fingerprint == self.file_fingerprint:
            return
        self.file_fingerprint = fingerprint
        data = {}
        try:
            data = json.loads(self.target_file.read_text(encoding="utf-8-sig"))
            target = Target.from_json(data, time.time(), self.p("command_max_age_s"))
            if target.request_id in self.seen:
                return
            self.seen.add(target.request_id)
            if self.active is not None:
                self.state["last_rejected_request_id"] = target.request_id
                self.record("request_rejected", {"request_id": target.request_id, "reason": "busy"})
                self.get_logger().warning(f"Rejected {target.request_id}: another command is active")
                return
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

    def start_plan(self):
        target = self.active
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.p("planning_group")
        motion.pipeline_id = self.p("pipeline_id")
        motion.planner_id = self.p("planner_id")
        motion.num_planning_attempts = 1
        motion.allowed_planning_time = float(self.p("planning_time_s"))
        motion.max_velocity_scaling_factor = float(self.p("velocity_scaling"))
        motion.max_acceleration_scaling_factor = float(self.p("acceleration_scaling"))
        motion.start_state.is_diff = True
        motion.start_state.joint_state = self.last_joint_msg
        motion.workspace_parameters.header.frame_id = "base"
        motion.workspace_parameters.min_corner.x = motion.workspace_parameters.min_corner.y = motion.workspace_parameters.min_corner.z = -2.0
        motion.workspace_parameters.max_corner.x = motion.workspace_parameters.max_corner.y = motion.workspace_parameters.max_corner.z = 2.0
        # Constrain the actual needle point on tool0. Constraining a flange point
        # computed with the target orientation would permit extra tip displacement
        # when MoveIt uses the allowed orientation tolerance.
        position = PositionConstraint()
        position.header.frame_id = "base"
        position.link_name = "tool0"
        position.target_point_offset.x, position.target_point_offset.y, position.target_point_offset.z = map(float, self.p("tip_offset_m"))
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(self.p("position_tolerance_m"))]
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [ros_pose(target.position_m, [0.0, 0.0, 0.0, 1.0])]
        orientation = OrientationConstraint()
        orientation.header.frame_id = "base"
        orientation.link_name = "tool0"
        orientation.orientation = ros_pose([0.0, 0.0, 0.0], target.orientation_xyzw).orientation
        # With rotation-vector parameterization, bounding each component by
        # global_budget/sqrt(3) also bounds the full rotation-vector norm.
        axis_tolerance = float(self.p("orientation_tolerance_rad")) / math.sqrt(3.0)
        orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = orientation.absolute_z_axis_tolerance = axis_tolerance
        orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.name = "needle_tip_target_v1"
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        motion.goal_constraints = [constraints]
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
            self.cached_target = target
            self.cached_trajectory = response.trajectory
            trajectory = response.trajectory.joint_trajectory
            self.state["plan"].update({"status": "succeeded", "request_id": target.request_id,
                                      "point_count": len(points), "planning_time_s": response.planning_time,
                                      "moveit_error_code": code, "tcp_path_m": [],
                                      "joint_trajectory": {"joint_names": list(trajectory.joint_names),
                                         "points": [{"positions_rad": list(p.positions), "time_from_start_s": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9} for p in points]}})
            self.record("plan_result", {"request_id": target.request_id, "point_count": len(points), "moveit_error_code": code, "planning_time_s": response.planning_time})
            self.preview_fk(target.request_id, trajectory)
            if target.command == "plan_execute":
                self.start_execute()
            else:
                self.state.update({"status": "planned", "message": "Trajectory planned; no execution requested"})
                self.active = None
        except Exception as exc:
            self.fail(f"Planning callback failed: {exc}")

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
        # Jazzy's official MoveIt yaml defaults to scaled JTC. Explicitly select the
        # ordinary JTC activated by mock.launch.py, avoiding that mismatch.
        goal.controller_names = [self.p("controller_name")]
        self.state.update({"status": "executing", "message": "Executing trajectory through MoveIt on mock hardware"})
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
            self.state.update({"status": "succeeded", "message": "MoveIt reports successful mock-hardware execution; inspect separately measured pose errors"})
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
            if node.goal_handle is not None and rclpy.ok():
                node.goal_handle.cancel_goal_async()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
