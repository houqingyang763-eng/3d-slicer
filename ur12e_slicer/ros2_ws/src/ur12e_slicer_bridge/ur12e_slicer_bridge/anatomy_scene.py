"""Synchronize the two static-head collision objects in the mock MoveIt scene."""
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
from geometry_msgs.msg import Point
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import String

from .anatomy_geometry import (ArmKinematics, CLEARANCE_MM, HeadClearance, JOINT_NAMES, STRUCTURES,
                               anatomy_archive, pose_matrix_mm, transform_points)
from .planning import ros_pose


OBJECT_IDS = {name: "ur12e_anatomy_" + name for name in STRUCTURES}
DEFAULT_POSITION_M = [-0.83, -0.018, 0.335]
IDENTITY = [0.0, 0.0, 0.0, 1.0]


class HeadScene:
    def __init__(self, bridge, root):
        self.bridge = bridge
        manifest, arrays = anatomy_archive(root)
        with arrays:
            self.clearance = HeadClearance(arrays)
            self.meshes = {}
            for name in STRUCTURES:
                points = arrays[name + "_planning_points_mm"].astype(float) / 1000
                faces = arrays[name + "_planning_faces"]
                self.meshes[name] = Mesh(vertices=[Point(x=x, y=y, z=z) for x, y, z in points],
                                         triangles=[MeshTriangle(vertex_indices=face.tolist()) for face in faces])
        self.path = bridge.runtime / "head_pose.json"
        self.tip_length_mm = float(bridge.p("tip_offset_m")[2]) * 1000
        self.get_client = bridge.create_client(GetPlanningScene, "/get_planning_scene")
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        bridge.create_subscription(String, "/robot_description", self.read_urdf, qos)
        self.kinematics = None
        self.kinematics_verified = False
        self.pending = False
        self.pending_since = 0.0
        self.retry_after = 0.0
        self.last_input = None
        self.known_revisions = set()
        session = uuid.uuid4().hex
        self.state = {
            "session": session, "revision": session, "confirmed_revision": None, "ready": False,
            "status": "waiting", "position_m": DEFAULT_POSITION_M.copy(), "orientation_xyzw": IDENTITY.copy(),
            "asset_sha256": manifest["geometry_sha256"], "subject": manifest["subject"],
            "margin_mm": CLEARANCE_MM, "planning_pad_mm": manifest["planning_pad_mm"],
            "frame_id": "base", "readback": None,
        }

    @property
    def revision(self):
        return self.state["revision"]

    @property
    def ready(self):
        return self.state["ready"]

    def read_urdf(self, message):
        try:
            self.kinematics = ArmKinematics(message.data, self.tip_length_mm)
            self.kinematics_verified = False
        except (ValueError, KeyError) as error:
            self.kinematics = None
            self.bridge.get_logger().error(f"Cannot prepare UR needle kinematics: {error}")

    def poll_pose(self):
        try:
            content = self.path.read_bytes()
        except FileNotFoundError:
            return False
        if self.last_input == content:
            return False
        self.last_input = content
        try:
            data = json.loads(content)
            if data.get("session") != self.state["session"]:
                return False
            revision = data["revision"]
            if not isinstance(revision, str) or uuid.UUID(hex=revision).hex != revision:
                raise ValueError("Head pose revision must be a UUID")
            if revision == self.revision or revision in self.known_revisions:
                raise ValueError("Head pose revision was already used")
            matrix = pose_matrix_mm(np.asarray(data["position_m"]) * 1000, data["orientation_xyzw"])
            if np.max(np.abs(matrix[:3, 3])) > 2000:
                raise ValueError("Head pose is outside the simulation workspace")
            self.known_revisions.add(revision)
            self.state.update(revision=revision, position_m=list(data["position_m"]),
                              orientation_xyzw=list(data["orientation_xyzw"]), status="waiting",
                              input_error=None, ready=False, readback=None)
        except (KeyError, ValueError, TypeError, AttributeError) as error:
            self.state.update(status="error", input_error=str(error), ready=False, readback=None)
        return True

    def base_from_head_mm(self):
        return pose_matrix_mm(np.asarray(self.state["position_m"]) * 1000, self.state["orientation_xyzw"])

    def update(self, link_transforms):
        if self.kinematics and not self.kinematics_verified and "tool0" in link_transforms:
            observed = link_transforms["tool0"]
            tf = pose_matrix_mm(np.asarray(observed["position_m"]) * 1000, observed["orientation_xyzw"])
            if all(name in self.bridge.joints for name in JOINT_NAMES):
                fk = self.kinematics.link_transform("tool0", [self.bridge.joints[name] for name in JOINT_NAMES])
                error_mm = float(np.linalg.norm(fk[:3, 3] - tf[:3, 3]))
                angle = math.acos(float(np.clip((np.trace(fk[:3, :3].T @ tf[:3, :3]) - 1) / 2, -1, 1)))
                self.state["fk_check"] = {"position_difference_mm": error_mm, "orientation_difference_rad": angle}
                self.kinematics_verified = error_mm <= 0.1 and angle <= 0.001
        if self.pending:
            if time.monotonic() - self.pending_since > 20:
                self.state.update(status="error", error="MoveIt anatomy update timed out", ready=False)
            return
        if (self.state.get("input_error") or not self.kinematics_verified or not self.bridge.scene_ready
                or not self.bridge.scene_client.service_is_ready() or not self.get_client.service_is_ready()):
            self.state["ready"] = False
            return
        if self.state["confirmed_revision"] == self.revision and self.state["readback"]:
            self.state.update(ready=True, status="ready")
            return
        if time.monotonic() < self.retry_after:
            return
        objects = [
            CollisionObject(id=OBJECT_IDS[name], pose=ros_pose(self.state["position_m"], self.state["orientation_xyzw"]),
                            meshes=[self.meshes[name]], mesh_poses=[ros_pose([0, 0, 0], IDENTITY)],
                            operation=CollisionObject.ADD)
            for name in STRUCTURES
        ]
        for item in objects:
            item.header.frame_id = "base"
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        self.pending = True
        self.pending_since = time.monotonic()
        revision = self.revision
        self.state.update(status="applying", ready=False, error=None)
        future = self.bridge.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        future.add_done_callback(lambda response: self.scene_applied(response, revision))

    def scene_applied(self, future, revision):
        try:
            if not future.result().success:
                raise RuntimeError("MoveIt rejected the anatomy scene")
            if revision != self.revision:
                self.pending = False
                return
            components = PlanningSceneComponents(
                components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
            request = GetPlanningScene.Request(components=components)
            self.get_client.call_async(request).add_done_callback(lambda response: self.readback_done(response, revision))
        except Exception as error:
            self.sync_failed(error)

    def pose_in_base(self, frame, pose):
        def matrix(position, orientation):
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            return pose_matrix_mm([position.x * 1000, position.y * 1000, position.z * 1000], q)
        if frame == "base":
            return matrix(pose.position, pose.orientation)
        transform = self.bridge.tf_buffer.lookup_transform("base", frame, Time()).transform
        return matrix(transform.translation, transform.rotation) @ matrix(pose.position, pose.orientation)

    def readback_done(self, future, revision):
        self.pending = False
        if revision != self.revision:
            return
        try:
            scene = future.result().scene
            objects = {item.id: item for item in scene.world.collision_objects}
            attached = {item.object.id: item for item in scene.robot_state.attached_collision_objects}
            needle = attached["slicer_simplified_needle"]
            if needle.link_name != "tool0" or not np.allclose(
                    needle.object.primitives[0].dimensions, [self.tip_length_mm / 1000, 0.003], atol=1e-10, rtol=0):
                raise RuntimeError("Needle collision geometry disagrees with the configured tool")
            expected = self.base_from_head_mm()
            checks = {}
            for name, identifier in OBJECT_IDS.items():
                item = objects[identifier]
                surface, = item.meshes
                pose, = item.mesh_poses
                matrix = self.pose_in_base(item.header.frame_id, item.pose) @ self.pose_in_base("base", pose)
                local = np.array([(v.x, v.y, v.z) for v in surface.vertices])
                original = np.array([(v.x, v.y, v.z) for v in self.meshes[name].vertices])
                faces = [list(face.vertex_indices) for face in surface.triangles]
                if (not np.allclose(matrix, expected, atol=1e-5, rtol=0) or local.shape != original.shape
                        or not np.allclose(local, original, atol=1e-8, rtol=0)
                        or faces != [list(face.vertex_indices) for face in self.meshes[name].triangles]):
                    raise RuntimeError(f"{name} collision mesh or transform did not match MoveIt readback")
                placed = transform_points(matrix, local * 1000)
                checks[name] = {"object_id": identifier, "frame": item.header.frame_id, "mesh_vertices": len(local),
                                "mesh_faces": len(faces), "T_base_head_mm": matrix.tolist(),
                                "bounds_base_mm": [placed.min(axis=0).tolist(), placed.max(axis=0).tolist()]}
            self.state.update(confirmed_revision=revision, readback=checks, status="ready", error=None,
                              ready=self.kinematics_verified and not self.state.get("input_error"))
            self.bridge.record("anatomy_scene_confirmed", {"revision": revision, "readback": checks})
        except Exception as error:
            self.sync_failed(error)

    def sync_failed(self, error):
        self.pending = False
        self.retry_after = time.monotonic() + 2
        self.state.update(status="error", error=str(error), ready=False, readback=None)
        self.bridge.get_logger().error(f"Anatomy scene synchronization failed: {error}")

    def goal_clearance(self, target):
        pose = pose_matrix_mm(np.asarray(target.position_m) * 1000, target.orientation_xyzw)
        pose[:3, 3] -= pose[:3, 2] * self.tip_length_mm
        return self.clearance.needle(pose, np.linalg.inv(self.base_from_head_mm()), self.tip_length_mm)
