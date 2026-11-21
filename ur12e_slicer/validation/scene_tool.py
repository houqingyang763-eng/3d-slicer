#!/usr/bin/env python3
"""Manage this demo's primitives using real MoveIt planning-scene services.

Run inside UR12e-Jazzy with ROS Jazzy sourced, ROS_DOMAIN_ID=62 and
ROS_LOCALHOST_ONLY=1. The JSON displayed by Slicer is a GetPlanningScene
readback, never the unconfirmed request used to add a box.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

PROJECT = Path(__file__).resolve().parents[1]
PREFIX = "ur12e_demo_"
PRIMITIVE_TYPES = {SolidPrimitive.BOX: "box", SolidPrimitive.SPHERE: "sphere"}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".scene.tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


def pose_json(pose):
    return {"position_m": [pose.position.x, pose.position.y, pose.position.z],
            "orientation_xyzw": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]}


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return [w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s]


def rotate(q, v):
    norm = math.sqrt(sum(c*c for c in q))
    x, y, z, w = [c/norm for c in q]
    vx, vy, vz = v
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx]


def compose(parent, child):
    delta = rotate(parent["orientation_xyzw"], child["position_m"])
    return {"position_m": [a+b for a, b in zip(parent["position_m"], delta)],
            "orientation_xyzw": multiply(parent["orientation_xyzw"], child["orientation_xyzw"])}


class SceneTool:
    def __init__(self, snapshot=None, *, prefix=PREFIX):
        if os.environ.get("ROS_DOMAIN_ID") != "62" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
            raise RuntimeError("Use this task's ROS_DOMAIN_ID=62 and ROS_LOCALHOST_ONLY=1")
        self.node = rclpy.create_node("ur12e_demo_scene_tool")
        self.apply_client = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_client = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.valid_client = self.node.create_client(GetStateValidity, "/check_state_validity")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.snapshot_path = Path(snapshot) if snapshot else PROJECT / "runtime" / "obstacle_scene.json"
        self.prefix = prefix
        self.last_scene = None

    def call(self, client, request, timeout=15):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"MoveIt service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"MoveIt service timed out: {client.srv_name}")
        return future.result()

    def base_pose(self, frame, pose):
        local = pose_json(pose)
        if frame == "base":
            return local
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                transform = self.tf.lookup_transform("base", frame, Time())
                t, q = transform.transform.translation, transform.transform.rotation
                return compose({"position_m": [t.x, t.y, t.z],
                                "orientation_xyzw": [q.x, q.y, q.z, q.w]}, local)
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        raise RuntimeError(f"Unable to express actual planning-scene object frame {frame!r} in base")

    def snapshot(self, operation="inspect"):
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                         PlanningSceneComponents.ROBOT_STATE |
                                         PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        self.last_scene = self.call(self.get_client, request).scene
        objects = []
        for item in self.last_scene.world.collision_objects:
            if not item.id.startswith(self.prefix):
                continue
            object_pose = self.base_pose(item.header.frame_id, item.pose)
            primitives = []
            for primitive, local in zip(item.primitives, item.primitive_poses):
                if primitive.type not in PRIMITIVE_TYPES:
                    raise RuntimeError("Unexpected primitive in the demo's MoveIt obstacle readback")
                primitives.append({"type": PRIMITIVE_TYPES[primitive.type], "dimensions_m": list(primitive.dimensions),
                                   "pose_base": compose(object_pose, pose_json(local))})
            objects.append({"id": item.id, "readback_frame": item.header.frame_id,
                            "readback_object_pose": pose_json(item.pose), "primitives": primitives})
        result = {"schema_version": 1, "source": "moveit_get_planning_scene", "timestamp": time.time(),
                  "frame_id": "base", "operation": operation, "simulation_only": True,
                  "objects": objects,
                  "world_object_ids": sorted(item.id for item in self.last_scene.world.collision_objects),
                  "attached_object_ids": [item.object.id for item in self.last_scene.robot_state.attached_collision_objects]}
        atomic_json(self.snapshot_path, result)
        return result

    def add_box(self, identifier, center, dimensions):
        if len(dimensions) != 3:
            raise ValueError("A box requires three dimensions")
        return self._add_primitive(identifier, center, SolidPrimitive.BOX, dimensions)

    def add_sphere(self, identifier, center, radius):
        return self._add_primitive(identifier, center, SolidPrimitive.SPHERE, [radius])

    def _add_primitive(self, identifier, center, primitive_type, dimensions):
        if not identifier.startswith(self.prefix):
            raise ValueError(f"Only {self.prefix} object IDs are permitted")
        if len(center) != 3 or not all(math.isfinite(x) for x in center):
            raise ValueError("Obstacle center must be a finite 3-vector")
        if not all(math.isfinite(x) and x > 0 for x in dimensions):
            raise ValueError("Obstacle dimensions must be finite and positive")
        item = CollisionObject()
        item.id, item.header.frame_id, item.operation = identifier, "base", CollisionObject.ADD
        item.pose.position.x, item.pose.position.y, item.pose.position.z = map(float, center)
        item.pose.orientation.w = 1.0
        shape = SolidPrimitive(type=primitive_type, dimensions=list(map(float, dimensions)))
        local = Pose()
        local.orientation.w = 1.0
        item.primitives, item.primitive_poses = [shape], [local]
        self.apply(item)
        readback = self.snapshot("add:"+identifier)
        own = next((obj for obj in readback["objects"] if obj["id"] == identifier), None)
        if not own or len(own["primitives"]) != 1:
            raise RuntimeError("Applied obstacle is absent from actual GetPlanningScene readback")
        primitive = own["primitives"][0]
        if (primitive["type"] != PRIMITIVE_TYPES[primitive_type] or
                len(primitive["dimensions_m"]) != len(dimensions) or
                math.dist(primitive["dimensions_m"], dimensions) > 1e-9 or
                math.dist(primitive["pose_base"]["position_m"], center) > 1e-8):
            raise RuntimeError("Actual obstacle readback differs from the requested base geometry")
        return readback

    def remove(self, identifier):
        if not identifier.startswith(self.prefix):
            raise ValueError(f"Only {self.prefix} object IDs are permitted")
        before = self.snapshot("inspect_before_remove:"+identifier)
        if not any(obj["id"] == identifier for obj in before["objects"]):
            return before
        item = CollisionObject()
        item.id, item.header.frame_id, item.operation = identifier, "base", CollisionObject.REMOVE
        self.apply(item)
        readback = self.snapshot("remove:"+identifier)
        if any(obj["id"] == identifier for obj in readback["objects"]):
            raise RuntimeError("MoveIt readback still contains the removed obstacle")
        return readback

    def apply(self, item):
        scene = PlanningScene()
        scene.is_diff, scene.robot_state.is_diff = True, True
        scene.world.collision_objects = [item]
        request = ApplyPlanningScene.Request(scene=scene)
        if not self.call(self.apply_client, request).success:
            raise RuntimeError("Real MoveIt ApplyPlanningScene rejected the change")

    def validity(self, joint_names, positions, *, group_name="ur_manipulator"):
        request = GetStateValidity.Request()
        request.group_name = group_name
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(joint_names)
        request.robot_state.joint_state.position = list(map(float, positions))
        response = self.call(self.valid_client, request)
        return {"valid": bool(response.valid),
                "contacts": [{"body_1": c.contact_body_1, "body_2": c.contact_body_2,
                              "depth_m": c.depth} for c in response.contacts]}

    def close(self):
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["inspect", "add", "remove"])
    parser.add_argument("--id", default=PREFIX+"side_box")
    parser.add_argument("--center", nargs=3, type=float)
    parser.add_argument("--dimensions", nargs=3, type=float, default=[0.06, 0.06, 0.12])
    parser.add_argument("--snapshot", type=Path)
    args = parser.parse_args()
    rclpy.init()
    tool = SceneTool(args.snapshot)
    try:
        if args.operation == "add":
            if args.center is None:
                parser.error("add requires --center X Y Z (base metres)")
            result = tool.add_box(args.id, args.center, args.dimensions)
        elif args.operation == "remove":
            result = tool.remove(args.id)
        else:
            result = tool.snapshot()
        print(json.dumps(result, indent=2))
    finally:
        tool.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
