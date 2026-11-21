#!/usr/bin/env python3
"""Acceptance through the live Slicer panel and the fixed UR12e mock graph."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
import uuid

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningOptions, PlanningScene, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.parameter import parameter_value_to_python
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scene_tool import PROJECT, SceneTool, atomic_json, pose_json
from ur12e_slicer_bridge.anatomy_geometry import (ArmKinematics, CLEARANCE_MM, HeadClearance, JOINT_NAMES,
                               anatomy_archive, pose_matrix_mm, transform_points)
from ur12e_slicer_bridge.anatomy_scene import DEFAULT_POSITION_M, OBJECT_IDS
from ur12e_slicer_bridge.planning import needle_plan_request, ros_pose
from ur12e_slicer_bridge.protocol import Target
from validate_keepout import PARAMETERS, joint_samples
from validate_ros_runtime import Validation, joint_delta

sys.path[:0] = [str(PROJECT / "bootstrap"), str(PROJECT / "slicer")]
from linux_lifecycle import backend_status, slicer_status
from task_client import send


def gui(code, timeout=30):
    prefix = "panel = slicer.ur12eSimulation\npanel.poll()\nhead = panel.anatomy\n"
    return send(prefix + code, timeout=timeout)["result"]


class AnatomyAcceptance(Validation):
    def __init__(self, args, scene):
        super().__init__(args)
        self.scene = scene
        self.scene.snapshot_path = self.output / "last_moveit_scene.json"
        self.fk_client = scene.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = scene.node.create_client(GetPositionIK, "/compute_ik")
        self.summary.update(mode="ur12e_mock_anatomy_plan_only", checks={}, scope="Static anatomy, piecewise-linear joint path")

    def write(self, name, value):
        atomic_json(self.output / (name + ".json"), value)

    def check(self, label, value):
        self.summary["checks"][label] = value
        atomic_json(self.output / "summary.json", self.summary)
        print(f"PASS {label}", flush=True)

    def screenshot(self, name, extra=""):
        relative = (self.output / f"{name}.png").relative_to(PROJECT).as_posix()
        gui(extra + f"\n_result = panel.capture(PROJECT / {relative!r})")

    def submit_gui(self, target=None):
        select = "head.demo_target()" if target is None else f"head.set_target({target!r})"
        request = gui(f"{select}\n_result = panel.send('plan')")
        if request is None:
            raise AssertionError("Normal Slicer planning button rejected the request")
        state, _, observed = self.wait_terminal(request)
        self.maximum_feedback_change = max(self.maximum_feedback_change,
                                           max(joint_delta(self.initial, frame) for frame in observed))
        assert state["execution"]["status"] == "none"
        return request, state

    def snapshot(self):
        scene = self.scene.snapshot()
        assert set(OBJECT_IDS.values()) <= set(scene["world_object_ids"]), "Active anatomical scene was modified"
        assert "slicer_simplified_needle" in scene["attached_object_ids"]
        return scene

    def move_group_plan_only(self, request):
        group = ActionClient(self.scene.node, MoveGroup, "/move_action")
        try:
            assert group.wait_for_server(timeout_sec=8), "MoveGroup planning action unavailable"
            excluded = [CollisionObject(id=identifier, operation=CollisionObject.REMOVE)
                        for identifier in OBJECT_IDS.values()]
            for item in excluded:
                item.header.frame_id = "base"
            diff = PlanningScene(is_diff=True)
            diff.robot_state.is_diff = True
            diff.world.collision_objects = excluded
            options = PlanningOptions(plan_only=True, planning_scene_diff=diff)
            goal = MoveGroup.Goal(request=request.motion_plan_request, planning_options=options)
            self.write("display_only_goal", message_to_ordereddict(goal))
            future = group.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.scene.node, future, timeout_sec=15)
            assert future.done() and future.result().accepted, "Display-only planning request was rejected"
            response = future.result().get_result_async()
            rclpy.spin_until_future_complete(self.scene.node, response, timeout_sec=30)
            assert response.done(), "Display-only planning request timed out"
            result = response.result()
            self.write("display_only_result", message_to_ordereddict(result.result))
            assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code.val == MoveItErrorCodes.SUCCESS
            assert not result.result.executed_trajectory.joint_trajectory.points
            return result.result.planned_trajectory.joint_trajectory
        finally:
            group.destroy()

    def load_robot(self):
        client = self.scene.node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        response = self.scene.call(client, GetParameters.Request(names=["robot_description"]))
        return ArmKinematics(response.values[0].string_value, self.tip_length_mm)

    def replay_samples(self, trajectory, kinematics, head_from_base):
        assert len(trajectory.points) >= 2
        names = list(trajectory.joint_names)
        first = {"joint_names": names, "joint_positions_rad": list(trajectory.points[0].positions)}
        assert joint_delta(self.initial, first) < 1e-5, "The comparison did not use the same start joints"
        order = [names.index(name) for name in JOINT_NAMES]
        rows = []
        for sample in joint_samples(trajectory):
            joints = [sample["positions_rad"][index] for index in order]
            tool = kinematics.link_transform("tool0", joints)
            witness = self.geometry.needle(tool, head_from_base, self.tip_length_mm)
            validity = self.scene.validity(names, sample["positions_rad"], group_name="")
            rows.append({**sample, **validity, "needle": witness})
        return rows

    def validate_planned(self, request, state, kinematics, head_from_base):
        self.write("protected_request", request)
        self.write("protected_state", state)
        plan = state["plan"]
        assert state["status"] == "planned" and plan["moveit_error_code"] == MoveItErrorCodes.SUCCESS, state["message"]
        report = json.loads((self.runtime / plan["head_report"]).read_text())
        self.write("protected_sweep", report)
        assert report["passed"] and report["certified_minimum_lower_mm"] >= CLEARANCE_MM
        assert report["whole_arm"]["passed"] and report["whole_arm"]["joint_step_rad"] <= .01
        assert report["head_revision"] == request["head_revision"]
        # Replay transforms at the closest frame must agree with the running MoveIt FK.
        witness = report["closest"]
        start = JointState(name=list(JOINT_NAMES), position=witness["joints_rad"])
        query = GetPositionFK.Request(fk_link_names=["tool0"], robot_state=RobotState(is_diff=True, joint_state=start))
        query.header.frame_id = "base"
        response = self.scene.call(self.fk_client, query)
        assert response.error_code.val == MoveItErrorCodes.SUCCESS and response.pose_stamped[0].header.frame_id == "base"
        moveit_pose = pose_json(response.pose_stamped[0].pose)
        frame = pose_matrix_mm(np.asarray(moveit_pose["position_m"]) * 1000, moveit_pose["orientation_xyzw"])
        assert np.allclose(frame, np.array(witness["links_base_mm"]["tool0"]), atol=1e-5, rtol=0)
        trajectory = plan["joint_trajectory"]
        msg = JointTrajectory(joint_names=trajectory["joint_names"],
                              points=[JointTrajectoryPoint(positions=point["positions_rad"]) for point in trajectory["points"]])
        safe = self.replay_samples(msg, kinematics, head_from_base)
        self.write("protected_samples", safe)
        assert all(row["valid"] for row in safe), "Full robot state validity rejected a protected trajectory sample"
        route_text = gui("_result = head.distance_label.text")
        assert "净距下界" in route_text and "全臂复核通过" in route_text, route_text
        self.check("protected_detour", {"full_robot_samples": len(safe), "clearance_samples": report["sample_count"],
                                        "covered_intervals": report["interval_count"],
                                        "minimum_lower_mm": report["certified_minimum_lower_mm"],
                                        "closest_sample_lower_mm": witness["lower_mm"],
                                        "closest_structure": witness["structure"],
                                        "selected_route": plan["head_search"]["selected_route"],
                                        "operation_wall_s": plan["head_search"]["operation_wall_s"], "panel_text": route_text})
        return report

    def transition_head(self, position_m, yaw_deg):
        values = [*[value * 1000 for value in position_m], yaw_deg]
        moved = gui(f"for spin, value in zip(head.head_position, {values!r}):\n    spin.setValue(value)\n"
                    "revision = head.apply_pose()\n"
                    "_result = {'revision': revision, 'previous_preview_hidden': not panel.plan_path.GetDisplayNode().GetVisibility()}")
        revision = moved["revision"]
        assert revision and moved["previous_preview_hidden"]
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            try:
                state = self.read_state()
            except (OSError, ValueError):
                time.sleep(.08)
                continue
            anatomy = state["anatomy"]
            if anatomy["confirmed_revision"] == revision and anatomy["ready"]:
                for data in anatomy["readback"].values():
                    matrix = np.asarray(data["T_base_head_mm"])
                    assert np.allclose(matrix[:3, 3], np.array(position_m) * 1000, atol=1e-5)
                    assert abs(math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])) - yaw_deg) < 1e-4
                return state
            time.sleep(.08)
        raise RuntimeError("Updated brain and skull transforms were not confirmed")

    def obstruction(self, name, local_tip_mm, target, base_from_head):
        goal = {"position_m": (transform_points(base_from_head, local_tip_mm) / 1000).tolist(),
                "orientation_xyzw": target["orientation_xyzw"]}
        request, rejected = self.submit_gui(goal)
        witness = rejected["plan"].get("clearance", {}).get("closest")
        self.write(name + "_request", request)
        self.write(name + "_state", rejected)
        assert rejected["status"] == "error" and witness and witness["interior_witness"], rejected["message"]
        assert rejected["plan"]["clearance"]["reason"] == "goal_blocked"
        assert not rejected["plan"]["point_count"]
        if name == "shaft_obstruction":
            assert witness["tip_inside"] == [] and witness["intersections"]["brain"]["shaft"]
        else:
            assert "brain" in witness["tip_inside"]

        # Generate a mock joint witness for the exact pose, without executing.
        pose = PoseStamped(pose=ros_pose(np.array(witness["tool_base_mm"]) / 1000, request["orientation_xyzw"]))
        pose.header.frame_id = "base"
        ik = PositionIKRequest(group_name="ur_manipulator", pose_stamped=pose, ik_link_name="tool0",
                               robot_state=RobotState(is_diff=True, joint_state=self.initial_joints))
        ik.timeout.nanosec = 500_000_000
        result = self.scene.call(self.ik_client, GetPositionIK.Request(ik_request=ik))
        self.write(name + "_ik", message_to_ordereddict(result))
        assert result.error_code.val == MoveItErrorCodes.SUCCESS, "Could not obtain a mock joint witness for the rejected pose"
        joints = result.solution.joint_state
        validity = self.scene.validity(joints.name, joints.position, group_name="")
        self.write(name + "_moveit_validity", validity)
        assert not validity["valid"] and any("slicer_simplified_needle" in (c["body_1"], c["body_2"]) and
                         set(OBJECT_IDS.values()) & {c["body_1"], c["body_2"]} for c in validity["contacts"]), (
                             "MoveIt did not report the needle touching an anatomical object")
        relative = (self.output / f"{name}.png").relative_to(PROJECT).as_posix()
        gui("import runpy\n"
            "reject_capture = runpy.run_path(str(PROJECT / 'validation/capture_anatomy.py'))['capture_rejection']\n"
            f"_result = reject_capture(panel, PROJECT / {relative!r}, {name == 'shaft_obstruction'!r})")
        self.check(name, {"witness": witness, "moveit_valid": validity["valid"],
                          "contacts": [list(pair) for pair in sorted({tuple(sorted((c["body_1"], c["body_2"])))
                                                                    for c in validity["contacts"]})]})

    def run(self):
        self.initial = self.wait_ready()
        assert self.initial["anatomy"]["ready"] and self.initial["anatomy"]["position_m"] == DEFAULT_POSITION_M, (
            "Use a newly started anatomy mock or restore the default head pose")
        self.write("initial_state", self.initial)
        self.write("initial_moveit_scene", self.snapshot())
        self.initial_joints = JointState(name=self.initial["joint_names"], position=self.initial["joint_positions_rad"])
        self.tip_length_mm = self.initial["tool"]["tip_offset_m"][2] * 1000
        self.maximum_feedback_change = 0.0
        manifest, archive = anatomy_archive(PROJECT / ".local/head_anatomy")
        self.write("source_manifest", manifest)
        with archive:
            self.geometry = HeadClearance(archive)
        kinematics = self.load_robot()
        head_state = self.initial["anatomy"]
        base_from_head = pose_matrix_mm(np.array(head_state["position_m"]) * 1000, head_state["orientation_xyzw"])
        head_from_base = np.linalg.inv(base_from_head)
        gui("panel.dock.setEnabled(False)\npanel.target_marker.SetLocked(True)\n_result = True")
        try:
            originals = gui("head.view.setCurrentIndex(1)\n_result = {'visible': head.mr_surface.GetDisplayNode().GetVisibility(),"
                            " 'node': head.mr_surface.GetName(), 'plan_held': not head.plan_enabled()}")
            assert originals["visible"] and originals["plan_held"]
            self.screenshot("original_mr")
            gui("head.view.setCurrentIndex(0)\n_result = True")
            visible = gui("_result = {name: bool(node.GetDisplayNode().GetVisibility()) for name, node in head.nodes.items()}")
            assert all(visible.values()) and set(visible) == set(OBJECT_IDS)
            self.check("head_layers_and_original_mr", {"nominal_layers_visible": visible, "original": originals,
                                                       "scene_readback": head_state["readback"]})

            request, protected = self.submit_gui()
            report = self.validate_planned(request, protected, kinematics, head_from_base)
            client = self.scene.node.create_client(GetParameters, "/ur12e_slicer_bridge/get_parameters")
            parameters = self.scene.call(client, GetParameters.Request(names=list(PARAMETERS)))
            settings = dict(zip(PARAMETERS, map(parameter_value_to_python, parameters.values)))
            target = Target.from_json(request, time.time(), 60)
            same_request = needle_plan_request(settings.__getitem__, self.initial_joints, target.position_m,
                                               target.orientation_xyzw)
            baseline = self.move_group_plan_only(same_request)
            self.snapshot()  # The diff was local to the plan-only action.
            samples = self.replay_samples(baseline, kinematics, head_from_base)
            self.write("display_only_in_active_scene", samples)
            assert samples[0]["valid"] and samples[-1]["valid"]
            collisions = [row for row in samples if not row["valid"]]
            assert any(row["needle"]["intersections"]["brain"]["shaft"] or row["needle"]["intersections"]["brain"]["tip"]
                       for row in collisions), "Old path lacks a collision through the labelled brain volume"
            contacts = {tuple(sorted((c["body_1"], c["body_2"]))) for row in collisions for c in row["contacts"]}
            assert any("slicer_simplified_needle" in pair and OBJECT_IDS["brain"] in pair for pair in contacts)
            self.check("same_start_target_baseline_collision", {"full_robot_samples": len(samples),
                       "invalid_samples": len(collisions), "contact_pairs": sorted(contacts)})

            paths = [row["needle"]["tip_base_mm"] for row in samples]
            relative = self.output.relative_to(PROJECT).as_posix()
            display = gui("import runpy\n"
                          "visualize = runpy.run_path(str(PROJECT / 'validation/capture_anatomy.py'))['capture']\n"
                          f"_result = visualize(panel, PROJECT / {relative!r}, {paths!r})", timeout=90)
            self.check("slicer_comparison_and_replay", display)

            shifted = [DEFAULT_POSITION_M[0], DEFAULT_POSITION_M[1], DEFAULT_POSITION_M[2] - .012]
            moved = self.transition_head(shifted, 10.0)
            self.write("moved_head_state", moved)
            assert moved["plan"]["status"] == "invalidated" and not moved["plan"]["tcp_path_m"]
            stale = {**request, "request_id": uuid.uuid4().hex, "timestamp": time.time()}
            atomic_json(self.runtime / "target.json", stale)
            stale_state, _, _ = self.wait_terminal(stale)
            assert stale_state["status"] == "error" and "stale or unconfirmed" in stale_state["message"]
            fresh_request, replanned = self.submit_gui({"position_m": request["position_m"], "orientation_xyzw": request["orientation_xyzw"]})
            self.write("replanned_after_head_move", replanned)
            assert replanned["status"] == "planned" and replanned["plan"]["clearance"]["passed"], replanned["message"]
            assert fresh_request["head_revision"] == moved["anatomy"]["revision"]
            self.check("head_pose_invalidates_old_plan", {"old_revision": request["head_revision"],
                       "new_revision": fresh_request["head_revision"], "replanned": True})
            self.transition_head(DEFAULT_POSITION_M, 0.0)
            for label, position in (("shaft_obstruction", [0.0, 0.0, -110.0]),
                                    ("goal_inside_brain", [0.0, 0.0, 34.0])):
                self.obstruction(label, position, request, base_from_head)
            self.screenshot("anatomy_scene")
            final = self.read_state()
            self.maximum_feedback_change = max(self.maximum_feedback_change, joint_delta(self.initial, final))
            events = [json.loads(line) for line in self.capture_events().splitlines() if line]
            assert not any(e["event"] == "execution_result" for e in events)
            assert self.maximum_feedback_change < 1e-5
            self.check("plan_only", {"maximum_joint_change_rad": self.maximum_feedback_change,
                                     "execution_result_events": 0})
            self.summary["passed"] = True
        finally:
            try:
                last = self.read_state()
                if last["anatomy"]["position_m"] != DEFAULT_POSITION_M or last["anatomy"]["orientation_xyzw"] != [0, 0, 0, 1]:
                    self.transition_head(DEFAULT_POSITION_M, 0.0)
            finally:
                gui("panel.dock.setEnabled(True)\npanel.target_marker.SetLocked(False)\n_result = True")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.set_defaults(runtime=PROJECT / "runtime", output=PROJECT / "output/anatomy_validation", ready_timeout=12.0,
                        step_timeout=35.0, max_feedback_age=1.0, position_tolerance=0.002, orientation_tolerance=0.03)
    args = parser.parse_args()
    status = backend_status()
    if not status.get("anatomy") or not status["ready"] or not slicer_status()["ready"]:
        parser.error("Start this project's mock and Slicer with start_linux.sh --anatomy.")
    with (args.runtime / "linux_lifecycle.lock").open("a") as lifecycle:
        fcntl.flock(lifecycle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (args.runtime / "validation.lock").exists():
            raise RuntimeError("A different validator controls the panel")
        lock_path = args.runtime / "navigation_validation.lock"
        with lock_path.open("x") as stream:
            json.dump({"pid": os.getpid(), "mode": "static_head"}, stream)
        try:
            rclpy.init(args=[])
            scene = SceneTool()
            try:
                proof = AnatomyAcceptance(args, scene)
                try:
                    proof.run()
                except Exception as error:
                    proof.summary.update(passed=False, error=str(error), traceback=traceback.format_exc())
                    print(f"FAIL {error}", file=sys.stderr)
                finally:
                    proof.finish()
                return 0 if proof.summary["passed"] else 1
            finally:
                scene.close()
                rclpy.shutdown()
        finally:
            lock_path.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
