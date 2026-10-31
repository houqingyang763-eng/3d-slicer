#!/usr/bin/env python3
"""Validate an already running, isolated UR12e URSim controller and MoveIt bridge.

No installation, container startup, dashboard command or physical-robot endpoint
is provided here. Run as an ordinary Linux user with the approved wrapper; keep
other file-bridge senders paused. Results describe a virtual controller only.
"""
import argparse
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

from validate_ros_runtime import (BUSY, REQUIRED_LINKS, Validation, atomic_json,
                                  joint_delta, normalized, pose_error, vector)


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
ROBOT_IP = "192.168.62.2"
SUBNET = "192.168.62.0/24"
GATEWAY = "192.168.62.1"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
HOME = [0.0, -1.3, 1.6, -1.9, -1.57, 0.0]
ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
JOINT_TOLERANCE = 0.02


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_environment():
    require(__debug__, "Run validation without Python -O; shared bridge assertions must remain enabled")
    require(platform.system() == "Linux", "URSim validation requires Linux")
    require(os.geteuid() != 0, "Run ROS validation as an ordinary user, not root")
    require(os.environ.get("ROS_DOMAIN_ID") == "62", "ROS_DOMAIN_ID must be 62")
    require(os.environ.get("ROS_LOCALHOST_ONLY") == "1", "ROS_LOCALHOST_ONLY must be 1")


def check_docker_identity(container, network, container_name, network_name, project_key):
    labels = {"ur12e_slicer.task": "ursim", "ur12e_slicer.project_key": project_key}
    expected_name = "ur12e-slicer-ursim-" + project_key
    require(container_name == network_name == expected_name, "Unexpected task-owned Docker names")
    require(container.get("Name", "").lstrip("/") == container_name, "Container name mismatch")
    require(network.get("Name") == network_name, "Network name mismatch")
    for resource, actual in (("container", container.get("Config", {}).get("Labels") or {}),
                             ("network", network.get("Labels") or {})):
        require(all(actual.get(k) == v for k, v in labels.items()), f"{resource} labels do not identify this task")
    image = container.get("Config", {}).get("Image", "")
    require(image.startswith(("universalrobots/ursim_e-series:",
                              "universalrobots/ursim_e-series@sha256:")), "Not the official URSim e-series image")
    require("ROBOT_MODEL=UR12" in container.get("Config", {}).get("Env", []), "URSim must use ROBOT_MODEL=UR12 for UR12e")
    require(container.get("State", {}).get("Running") is True, "URSim container is not running")
    require(network.get("Internal") is True and network.get("Driver") == "bridge", "URSim network must be an internal Docker bridge")
    configs = network.get("IPAM", {}).get("Config", [])
    require(len(configs) == 1 and configs[0].get("Subnet") == SUBNET and
            configs[0].get("Gateway") == GATEWAY, "URSim subnet/gateway mismatch")
    attached = container.get("NetworkSettings", {}).get("Networks", {})
    require(set(attached) == {network_name}, "URSim must only attach to its isolated task network")
    endpoint = attached[network_name]
    require(endpoint.get("IPAddress") == ROBOT_IP and endpoint.get("NetworkID") == network.get("Id"), "URSim endpoint IP/network identity mismatch")
    members = network.get("Containers") or {}
    own_member = members.get(container.get("Id"), {})
    require(len(members) == 1 and own_member.get("Name") == container_name and
            own_member.get("IPv4Address") == ROBOT_IP + "/24", "Unexpected network members or endpoint address")


def check_robot_description(xml):
    """Reject a ROS graph configured for mock or any other controller IP."""
    root = ET.fromstring(xml)
    hardware = root.findall(".//ros2_control/hardware")
    require(len(hardware) == 1, "Expected exactly one ros2_control hardware definition")
    plugin = (hardware[0].findtext("plugin") or "").strip()
    require(plugin == "ur_robot_driver/URPositionHardwareInterface", "ROS hardware is not the UR driver")
    params = {p.get("name"): (p.text or "").strip() for p in hardware[0].findall("param")}
    require(params.get("robot_ip") == ROBOT_IP, "ROS driver robot_ip is not the verified URSim endpoint")


def check_state(state, file_mtime, now, max_age=1.0):
    require(state.get("source") == "ros2_ursim", "Refusing a backend other than ros2_ursim")
    require(type(state.get("schema_version")) is int and state["schema_version"] == 1, "Unexpected state schema")
    stamp = state.get("timestamp")
    require(type(stamp) in (int, float) and math.isfinite(stamp) and
            -0.5 <= now - stamp <= max_age, "URSim backend state is stale or future-dated")
    require(-0.5 <= now - file_mtime <= max_age, "URSim state file is stale or future-dated")
    require(state.get("feedback_valid") is True and state.get("ready") is True, "URSim backend is not fresh and ready")
    for field in ("joint_state_age_s", "tf_age_s"):
        age = state.get(field)
        require(type(age) in (int, float) and math.isfinite(age) and 0 <= age < max_age, f"Invalid/stale {field}")
    require(len(state.get("joint_names", [])) == 6 and set(state["joint_names"]) == set(JOINTS), "UR12e joint names are incomplete")
    vector(state.get("joint_positions_rad"), 6)
    require(state.get("needle_collision_ready") is True, "Needle collision geometry is not ready")
    require(REQUIRED_LINKS <= set(state.get("link_transforms", {})), "Incomplete robot/needle TF")
    tcp = state.get("tcp_pose")
    require(isinstance(tcp, dict) and tcp.get("frame_id") == "base", "Needle TCP must be in base")
    vector(tcp.get("position_m"), 3)
    normalized(tcp.get("orientation_xyzw"))
    distance, angle = pose_error(tcp, state["link_transforms"]["needle_tip"])
    require(distance < 1e-9 and angle < 1e-7, "TCP and needle-tip transform disagree")


class RosSession:
    """Small synchronous rclpy client; imports stay optional for offline tests."""
    def __init__(self, validation):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rcl_interfaces.srv import GetParameters
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        self.rclpy = rclpy
        self.action_type = FollowJointTrajectory
        self.parameter_type = GetParameters
        self.validation = validation
        self.latest = None
        self.last_logged = 0.0
        self.joint_count = 0
        self.feedback_count = 0
        self.active_goal = None
        self.active_result = None
        self.pending_goal = None
        rclpy.init(args=[])
        self.node = rclpy.create_node("ur12e_ursim_validation_" + uuid.uuid4().hex[:8])
        self.subscription = self.node.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.action = ActionClient(self.node, FollowJointTrajectory, ACTION)
        self.parameters = self.node.create_client(GetParameters, "/robot_state_publisher/get_parameters")

    def on_joints(self, msg):
        if not set(JOINTS) <= set(msg.name):
            return
        require(len(msg.name) == len(set(msg.name)) and len(msg.position) == len(msg.name), "Malformed /joint_states names/positions")
        by_name = dict(zip(msg.name, msg.position))
        positions = [by_name[n] for n in JOINTS]
        vector(positions, 6)
        now = time.monotonic()
        self.latest = {"received_at": time.time(), "received_monotonic": now,
                       "stamp_s": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                       "joint_names": JOINTS, "joint_positions_rad": positions}
        self.joint_count += 1
        if now - self.last_logged >= 0.05:
            self.validation.log("joint_states", self.latest)
            self.last_logged = now

    def spin(self, timeout=0.02):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def wait_future(self, future, timeout, label, monitor=None):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.spin()
            if monitor:
                monitor()
        require(future.done(), f"Timed out waiting for {label}")
        require(future.exception() is None, f"{label}: {future.exception()}")
        return future.result()

    def fresh_joints(self, after=None, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin()
            if self.latest is None:
                continue
            now = time.monotonic()
            stamp_age = self.node.get_clock().now().nanoseconds * 1e-9 - self.latest["stamp_s"]
            if (now - self.latest["received_monotonic"] <= 1.0 and -0.5 <= stamp_age <= 1.0 and
                    (after is None or self.latest["received_monotonic"] > after)):
                return dict(self.latest)
        raise RuntimeError("No fresh actual /joint_states received after the requested boundary")

    def verify_description(self):
        require(self.parameters.wait_for_service(timeout_sec=15), "robot_state_publisher parameter service missing")
        request = self.parameter_type.Request()
        request.names = ["robot_description"]
        response = self.wait_future(self.parameters.call_async(request), 15, "robot_description")
        require(len(response.values) == 1 and response.values[0].type == 4, "Missing string robot_description")
        xml = response.values[0].string_value
        (self.validation.output / "robot_description.urdf").write_text(xml, encoding="utf-8")
        check_robot_description(xml)
        require(self.action.wait_for_server(timeout_sec=15), f"Missing actual ROS action {ACTION}")

    def cancel_owned_goal(self):
        # send_goal_async may be accepted after a timeout or monitor failure.
        # Recover that handle before releasing this client's ROS resources.
        if self.pending_goal is not None:
            try:
                handle = self.wait_future(self.pending_goal, 8, "pending goal acceptance during cleanup")
                self.pending_goal = None
                if handle.accepted:
                    self.active_goal = handle
                    self.active_result = handle.get_result_async()
            except Exception as exc:
                self.validation.log("pending_goal_unresolved", {"error": str(exc)})
                raise RuntimeError("Could not resolve this client's pending URSim goal; its termination is unconfirmed") from exc
        if self.active_goal is None:
            return
        try:
            result = self.wait_future(self.active_goal.cancel_goal_async(), 5, "own URSim goal cancellation")
            self.validation.log("cancel_goal", {"return_code": result.return_code,
                                               "goals_canceling": len(result.goals_canceling)})
            terminal = self.wait_future(self.active_result, 8, "canceled URSim goal terminal result")
            self.validation.log("cancel_goal_terminal", {"action_status": terminal.status,
                "controller_error_code": terminal.result.error_code,
                "controller_error_string": terminal.result.error_string})
            require(terminal.status in (4, 5, 6), "Canceled URSim goal has no confirmed terminal status")
        except Exception as exc:
            self.validation.log("cancel_goal_failed", {"error": str(exc)})
            raise RuntimeError("Could not confirm termination of this client's URSim goal") from exc
        self.active_goal = None
        self.active_result = None

    def move(self, label, positions, duration):
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        before = self.fresh_joints()
        baseline = self.validation.wait_ready()
        require(baseline.get("status") not in BUSY, "Bridge is already controlling the robot")
        goal = self.action_type.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        start = JointTrajectoryPoint()
        start.positions = before["joint_positions_rad"]
        start.velocities = [0.0] * 6
        end = JointTrajectoryPoint()
        end.positions = list(positions)
        end.velocities = [0.0] * 6
        end.time_from_start.sec = int(duration)
        end.time_from_start.nanosec = round((duration - int(duration)) * 1e9)
        goal.trajectory.points = [start, end]
        goal.goal_tolerance = [JointTolerance(name=name, position=JOINT_TOLERANCE) for name in JOINTS]
        goal.goal_time_tolerance.sec = 5
        request = {"action": ACTION, "joint_names": JOINTS, "from_rad": list(start.positions),
                   "target_rad": list(positions), "duration_s": duration, "goal_tolerance_rad": JOINT_TOLERANCE}
        case = {"name": label, "kind": "FollowJointTrajectory", "passed": False, **request}
        self.validation.summary["cases"].append(case)
        self.validation.save_summary()
        atomic_json(self.validation.output / (label + "_goal.json"), request)
        last_feedback = [0.0]

        def feedback(message):
            self.feedback_count += 1
            if time.monotonic() - last_feedback[0] >= 0.05:
                f = message.feedback
                self.validation.log("action_feedback", {"case": label, "joint_names": list(f.joint_names),
                    "desired_rad": list(f.desired.positions), "actual_rad": list(f.actual.positions),
                    "error_rad": list(f.error.positions)})
                last_feedback[0] = time.monotonic()

        def monitor():
            current = self.validation.read_state()
            require(current.get("status") not in BUSY and current.get("request_id") == baseline.get("request_id"),
                    "Another file-bridge sender became active during direct joint control")
            require(self.latest is not None and time.monotonic() - self.latest["received_monotonic"] <= 1,
                    "Actual joint feedback became stale during URSim trajectory")
            stamp_age = self.node.get_clock().now().nanoseconds * 1e-9 - self.latest["stamp_s"]
            require(-0.5 <= stamp_age <= 1.0, "Actual joint sample timestamp became stale during URSim trajectory")

        self.pending_goal = self.action.send_goal_async(goal, feedback_callback=feedback)
        handle = self.wait_future(self.pending_goal, 15, "trajectory goal acceptance", monitor)
        self.pending_goal = None
        case["accepted"] = bool(handle.accepted)
        require(handle.accepted, f"{label}: URSim controller rejected trajectory")
        self.active_goal = handle
        self.active_result = handle.get_result_async()
        result = self.wait_future(self.active_result, self.validation.args.step_timeout, "trajectory result", monitor)
        self.active_goal = None
        self.active_result = None
        case.update({"action_status": result.status, "controller_error_code": result.result.error_code,
                     "controller_error_string": result.result.error_string})
        self.validation.save_summary()
        require(result.status == 4 and result.result.error_code == 0, f"{label}: FollowJointTrajectory failed: {case}")
        actual = self.fresh_joints(after=time.monotonic())
        errors = [abs(a - b) for a, b in zip(actual["joint_positions_rad"], positions)]
        movement = max(abs(a - b) for a, b in zip(actual["joint_positions_rad"], before["joint_positions_rad"]))
        case.update({"actual_joint_state": actual, "absolute_joint_errors_rad": errors,
                     "max_joint_error_rad": max(errors), "observed_joint_change_rad": movement})
        atomic_json(self.validation.output / (label + "_result.json"), case)
        require(max(errors) <= JOINT_TOLERANCE, f"{label}: actual joints exceed 0.02 rad error")
        if label in ("02_joint1_plus_0p05", "03_return_home"):
            require(movement >= 0.025, f"{label}: displaced command has no observed actual movement")
        case["passed"] = True
        atomic_json(self.validation.output / (label + "_result.json"), case)
        self.validation.save_summary()
        print(f"PASS {label}: actual max joint error {max(errors):.6f} rad", flush=True)

    def close(self):
        try:
            self.cancel_owned_goal()
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()


class UrsimValidation(Validation):
    def __init__(self, args):
        super().__init__(args)
        self.summary.update({"mode": "ursim_virtual_controller_basic_control", "source": "ros2_ursim",
                             "boundary": "URSim virtual controller, not mock hardware, physical robot, optical tracking or ASTM performance.",
                             "robot_ip": ROBOT_IP, "container": args.container, "network": args.network,
                             "joint_tolerance_rad": JOINT_TOLERANCE})
        self.ros = None
        self.evidence = (self.output / "ursim_events.jsonl").open("w", encoding="utf-8")
        self.save_summary()

    def save_summary(self):
        atomic_json(self.output / "summary.json", self.summary)

    def log(self, event, detail):
        self.evidence.write(json.dumps({"event": event, "timestamp": time.time(), **detail}, allow_nan=False) + "\n")
        self.evidence.flush()

    def docker_json(self, arguments, filename):
        result = subprocess.run(["docker", *arguments], capture_output=True, text=True, timeout=20, check=False)
        (self.output / (filename + ".stderr.log")).write_text(result.stderr, encoding="utf-8")
        (self.output / (filename + ".json")).write_text(result.stdout, encoding="utf-8")
        self.log("docker_inspect", {"arguments": arguments, "return_code": result.returncode})
        require(result.returncode == 0, "Docker identity inspection failed: " + result.stderr.strip())
        return json.loads(result.stdout)

    def verify_docker(self):
        # A remote Docker context would not identify this machine's URSim.
        docker_host = os.environ.get("DOCKER_HOST", "")
        require(not docker_host or docker_host.startswith("unix:///"), "Remote DOCKER_HOST is not allowed")
        context = self.docker_json(["context", "inspect"], "docker_context")
        require(len(context) == 1 and context[0].get("Endpoints", {}).get("docker", {}).get("Host", "").startswith("unix:///"),
                "Docker context must use a local Unix socket")
        container = self.docker_json(["container", "inspect", self.args.container], "docker_container")
        network = self.docker_json(["network", "inspect", self.args.network], "docker_network")
        require(len(container) == len(network) == 1, "Docker identity is ambiguous")
        key = hashlib.sha256(str(REPO.resolve()).encode("utf-8")).hexdigest()[:12]
        check_docker_identity(container[0], network[0], self.args.container, self.args.network, key)
        self.summary["project_key"] = key
        self.save_summary()

    def read_state(self):
        if self.ros:
            self.ros.spin(timeout=0)
        # DrvFS can briefly hide a file during atomic replacement. Retry only
        # storage visibility errors, within one total 250 ms budget. Parsing,
        # source, validity and freshness failures below are never retried.
        read_started = time.monotonic()
        deadline = read_started + 0.25
        retryable = {errno.ENOENT, errno.EACCES, errno.EPERM, getattr(errno, "ENODATA", 61)}
        while True:
            try:
                raw = self.state_path.read_text(encoding="utf-8-sig")
                file_mtime = self.state_path.stat().st_mtime
                break
            except OSError as exc:
                remaining = deadline - time.monotonic()
                if exc.errno not in retryable or remaining <= 0:
                    raise
                self.log("state_storage_retry", {"errno": exc.errno, "error": str(exc),
                                                  "remaining_budget_s": remaining})
                time.sleep(min(0.01, remaining))
                if time.monotonic() >= deadline:
                    raise
        state = json.loads(raw)
        checked_at = time.time()
        try:
            check_state(state, file_mtime, checked_at, self.args.max_feedback_age)
        except Exception as exc:
            if "first_rejected_state" not in self.summary:
                stamp = state.get("timestamp") if isinstance(state, dict) else None
                finite_stamp = type(stamp) in (int, float) and math.isfinite(stamp)
                # Preserve the original text as well as a JSON-safe decoded
                # snapshot, so nonfinite rejected inputs cannot hide the error
                # by making the diagnostic writer fail first.
                diagnostic = {
                    "error": str(exc), "error_type": type(exc).__name__,
                    "now": checked_at,
                    "state_age_s": checked_at - stamp if finite_stamp else None,
                    "file_age_s": checked_at - file_mtime,
                    "read_elapsed_monotonic_s": time.monotonic() - read_started,
                    "raw_state_json": raw,
                    "state": json.loads(raw, parse_constant=lambda value: {"nonfinite": value}),
                }
                self.summary["first_rejected_state"] = diagnostic
                try:
                    atomic_json(self.output / "rejected_state.json", diagnostic)
                    self.save_summary()
                except Exception as diagnostic_error:
                    self.summary["rejected_state_write_error"] = str(diagnostic_error)
            raise
        if state["timestamp"] != self.last_trace_timestamp:
            self.trace.write(json.dumps(state, allow_nan=False) + "\n")
            self.trace.flush()
            self.last_trace_timestamp = state["timestamp"]
        return state

    def wait_ready(self):
        deadline = time.monotonic() + self.args.ready_timeout
        error = "No state"
        while time.monotonic() < deadline:
            try:
                state = self.read_state()
                if state.get("status") not in BUSY:
                    return state
                error = "Bridge is busy: " + str(state.get("status"))
            except (OSError, ValueError, RuntimeError, AssertionError) as exc:
                error = str(exc)
            time.sleep(0.05)
        raise RuntimeError("URSim bridge not ready: " + error)

    def run(self):
        self.summary["current_step"] = "environment_and_identity"
        check_environment()
        self.verify_docker()
        initial = self.wait_ready()
        atomic_json(self.output / "initial_state.json", initial)
        self.ros = RosSession(self)
        self.ros.verify_description()
        self.summary["current_step"] = "01_bent_elbow_home"
        self.ros.move("01_bent_elbow_home", HOME, 15.0)
        moved = list(HOME)
        moved[0] += 0.05
        self.summary["current_step"] = "02_joint1_plus_0p05"
        self.ros.move("02_joint1_plus_0p05", moved, 3.0)
        self.summary["current_step"] = "03_return_home"
        self.ros.move("03_return_home", HOME, 3.0)
        # Inspect the same container immediately before the file bridge can act.
        self.summary["current_step"] = "verify_docker_before_bridge"
        self.verify_docker()
        before = self.wait_ready()
        self.summary["current_step"] = "04_tip_plus_z10mm_plan"
        target = {"position_m": [p + d for p, d in zip(before["tcp_pose"]["position_m"], (0, 0, 0.010))],
                  "orientation_xyzw": list(before["tcp_pose"]["orientation_xyzw"])}
        plan = self.submit("plan", target, "04_tip_plus_z10mm_plan")
        planned, statuses, states = self.wait_terminal(plan)
        require(planned["status"] == "planned", "Actual MoveIt planning failed: " + str(planned.get("message")))
        require(planned.get("plan", {}).get("moveit_error_code") == 1 and planned["plan"].get("point_count", 0) >= 2,
                "No usable actual MoveIt trajectory")
        planned = self.wait_preview(plan["request_id"], states)
        no_motion = max(joint_delta(before, s) for s in states + [planned])
        self.save_case("04_tip_plus_z10mm_plan", plan, planned, observed_statuses=statuses,
                       max_joint_change_rad=no_motion, passed=no_motion <= 0.001)
        require(no_motion <= 0.001, "Plan-only changed actual joints")
        self.summary["current_step"] = "05_tip_plus_z10mm_execute"
        execute = self.submit("execute", target, "05_tip_plus_z10mm_execute", plan["request_id"])
        terminal, statuses, observations = self.wait_terminal(execute)
        require(terminal.get("plan", {}).get("request_id") == plan["request_id"], "MoveIt executed a different cached plan")
        final = self.check_execution("05_tip_plus_z10mm_execute", execute, planned, terminal, statuses, observations)
        actual = self.ros.fresh_joints(after=time.monotonic())
        bridge_joints = dict(zip(final["joint_names"], final["joint_positions_rad"]))
        difference = max(abs(bridge_joints[n] - p) for n, p in zip(JOINTS, actual["joint_positions_rad"]))
        require(difference <= JOINT_TOLERANCE, "Bridge final joints disagree with fresh actual /joint_states")
        self.summary["final_actual_joint_state"] = actual
        self.summary["bridge_to_actual_joint_difference_rad"] = difference
        events = [json.loads(line) for line in self.capture_events().splitlines() if line.strip()]
        require(any(e.get("event") == "request_accepted" and e.get("request_id") == plan["request_id"] for e in events), "Missing actual bridge plan event")
        require(any(e.get("event") == "execution_result" and e.get("request_id") == execute["request_id"] and
                    e.get("action_status") == 4 and e.get("moveit_error_code") == 1 for e in events), "Missing actual MoveIt action-result event")
        self.summary["joint_messages_received"] = self.ros.joint_count
        self.summary["action_feedback_messages_received"] = self.ros.feedback_count
        self.summary["current_step"] = "completed"
        self.summary["passed"] = True

    def finish(self):
        try:
            if self.ros:
                self.ros.close()
        except Exception as exc:
            self.summary["cleanup_error"] = str(exc)
            self.summary["passed"] = False
        finally:
            try:
                try:
                    super().finish()
                except Exception as exc:
                    self.summary["finish_error"] = str(exc)
                    self.summary["passed"] = False
                    self.summary["finished_at"] = time.time()
                    self.save_summary()
                    self.trace.close()
                    print(f"Results: {self.output}", flush=True)
            finally:
                self.evidence.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=ROOT / "runtime")
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "ursim_validation")
    parser.add_argument("--container", required=True)
    parser.add_argument("--network", required=True)
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.add_argument("--step-timeout", type=float, default=180.0)
    args = parser.parse_args(argv)
    if any(not math.isfinite(v) or v <= 0 for v in (args.ready_timeout, args.step_timeout)):
        parser.error("timeouts must be finite and positive")
    args.position_tolerance = 0.002
    args.orientation_tolerance = 0.03
    args.max_feedback_age = 1.0
    gate = UrsimValidation(args)
    lock = args.runtime.resolve() / "validation.lock"
    token = uuid.uuid4().hex
    owns_lock = False
    try:
        check_environment()
        require(args.runtime.resolve().is_dir(), "Runtime directory does not exist")
        with lock.open("x", encoding="utf-8") as handle:
            json.dump({"pid": os.getpid(), "run_id": gate.run_id, "token": token, "created_at": time.time()}, handle)
        owns_lock = True
        gate.run()
    except BaseException as exc:
        gate.summary["error"] = str(exc) or type(exc).__name__
        gate.summary["traceback"] = traceback.format_exc()
        print("FAIL: " + gate.summary["error"], flush=True)
    finally:
        try:
            gate.finish()
        finally:
            if owns_lock and lock.exists():
                require(json.loads(lock.read_text(encoding="utf-8")).get("token") == token, "Validation lock ownership changed; retaining lock")
                lock.unlink()
    return 0 if gate.summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
