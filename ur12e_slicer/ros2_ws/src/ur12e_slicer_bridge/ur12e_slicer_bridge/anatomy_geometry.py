"""Geometry shared by the static-head planner, replay, and acceptance tests."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
STRUCTURES = ("brain", "skull")
NEEDLE_RADIUS_MM = 3.0
CLEARANCE_MM = 5.0
CORNERS = np.array(list(np.ndindex(2, 2, 2)))


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def anatomy_archive(folder):
    root = Path(folder)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    archive = root / "geometry.npz"
    if manifest["schema_version"] != 1 or sha256(archive) != manifest["geometry_sha256"]:
        raise ValueError("Prepared head geometry hash/schema mismatch")
    return manifest, np.load(archive, allow_pickle=False)


def rotation(axis, angle):
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def pose_matrix_mm(position_mm, orientation_xyzw):
    xyz = np.asarray(position_mm, dtype=float)
    q = np.asarray(orientation_xyzw, dtype=float)
    if (xyz.shape != (3,) or q.shape != (4,) or not np.isfinite(xyz).all()
            or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.0) > 1e-6):
        raise ValueError("Expected a finite rigid pose with a unit xyzw quaternion")
    v = q[:3]
    cross = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    transform = np.eye(4)
    transform[:3, :3] += 2 * (q[3] * cross + cross @ cross)
    transform[:3, 3] = xyz
    return transform


def transform_points(matrix, points):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


@dataclass
class Joint:
    name: str
    child: str
    parent: str
    origin: np.ndarray
    axis: np.ndarray
    actuated: bool


class ArmKinematics:
    """URDF FK in base and a per-joint upper bound on needle-point travel."""

    def __init__(self, urdf, tip_length_mm):
        self.tip_length_mm = tip_length_mm
        self.links = {}
        for element in ET.fromstring(urdf).findall("joint"):
            kind = element.attrib["type"]
            parent = element.find("parent").attrib["link"]
            child = element.find("child").attrib["link"]
            origin = element.find("origin")
            attributes = origin.attrib if origin is not None else {}
            xyz = np.fromstring(attributes.get("xyz", "0 0 0"), sep=" ") * 1000
            roll, pitch, yaw = np.fromstring(attributes.get("rpy", "0 0 0"), sep=" ")
            matrix = np.eye(4)
            matrix[:3, :3] = rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)
            matrix[:3, 3] = xyz
            axis_element = element.find("axis")
            axis = np.fromstring(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0", sep=" ")
            if kind not in {"fixed", "revolute", "continuous"} or element.find("mimic") is not None:
                raise ValueError(f"Unsupported URDF joint for needle sweep: {element.attrib['name']}")
            if axis.shape != (3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis) - 1) > 1e-6:
                raise ValueError("Invalid URDF joint axis")
            self.links[child] = Joint(element.attrib["name"], child, parent, matrix, axis, kind != "fixed")

        tool_chain, tool_root = self.chain("tool0")
        base_chain, base_root = self.chain("base")
        actuated = tuple(joint.name for joint in tool_chain if joint.actuated)
        if tool_root != base_root or actuated != JOINT_NAMES or any(joint.actuated for joint in base_chain):
            raise ValueError("Expected the fixed UR base and six ordered actuated joints from its root to tool0")
        self.tool_chain = tool_chain
        self.root = tool_root
        self.base_from_root = np.linalg.inv(self.forward_chain(base_chain, {})[0])
        reach = tip_length_mm + NEEDLE_RADIUS_MM
        bounds = {}
        for joint in reversed(tool_chain):
            if joint.actuated:
                bounds[joint.name] = reach
            reach += np.linalg.norm(joint.origin[:3, 3])
        self.reach_mm = np.array([bounds[name] for name in JOINT_NAMES])

    def chain(self, link):
        joints = []
        visited = set()
        while link in self.links:
            if link in visited:
                raise ValueError("URDF contains a joint cycle")
            visited.add(link)
            joint = self.links[link]
            joints.append(joint)
            link = joint.parent
        return list(reversed(joints)), link

    @staticmethod
    def forward_chain(chain, values):
        matrix = np.eye(4)
        links = {}
        for joint in chain:
            matrix = matrix @ joint.origin
            if joint.actuated:
                matrix[:3, :3] = matrix[:3, :3] @ rotation(joint.axis, values[joint.name])
            links[joint.child] = matrix.copy()
        return matrix, links

    def transforms(self, positions):
        _, frames = self.forward_chain(self.tool_chain, dict(zip(JOINT_NAMES, positions, strict=True)))
        return {link: self.base_from_root @ matrix for link, matrix in frames.items()}

    def link_transform(self, name, positions):
        chain, root = self.chain(name)
        if root != self.root:
            raise ValueError(f"URDF link {name} is outside the robot tree")
        return self.base_from_root @ self.forward_chain(chain, dict(zip(JOINT_NAMES, positions, strict=True)))[0]


class HeadClearance:
    """Distance lower bounds for two labelled solids and their display surfaces."""

    def __init__(self, arrays):
        self.origin = np.asarray(arrays["grid_origin_mm"], dtype=float)
        self.spacing = float(arrays["grid_spacing_mm"])
        self.distance = np.stack([arrays[name + "_distance_mm"] for name in STRUCTURES])
        # Each point of an occupied voxel or marching-cubes cell lies within
        # one cell diagonal of some occupied centre. Allow 0.05 mm for rounding.
        self.cell_cover = math.sqrt(3) * self.spacing + 0.05
        self.shape = np.array(self.distance.shape[1:])
        if self.spacing <= 0 or self.origin.shape != (3,) or not np.isfinite(self.distance).all():
            raise ValueError("Invalid head distance data")
        boxes = []
        for field in self.distance:
            cells = np.argwhere(field == 0)
            minimum = self.origin + cells.min(axis=0) * self.spacing - self.cell_cover
            maximum = self.origin + cells.max(axis=0) * self.spacing + self.cell_cover
            boxes.append((minimum, maximum))
        self.boxes = np.array(boxes)

    def point_bounds(self, points_head_mm):
        points = np.asarray(points_head_mm)
        grid = (points - self.origin) / self.spacing
        neighbours = np.clip(np.floor(grid).astype(int)[:, None, :] + CORNERS, 0, self.shape - 1)
        distances = self.distance[:, neighbours[..., 0], neighbours[..., 1], neighbours[..., 2]]
        offsets = np.linalg.norm(neighbours - grid[:, None, :], axis=2) * self.spacing
        # Distance to the set of occupied centres is 1-Lipschitz.
        lower = np.maximum(0, (distances - offsets).max(axis=2) - self.cell_cover)
        # The enclosing boxes give a second lower bound outside the sampled grid.
        box_min, box_max = self.boxes[:, 0, None, :], self.boxes[:, 1, None, :]
        to_box = np.linalg.norm(np.maximum(0, np.maximum(box_min - points, points - box_max)), axis=2)
        lower = np.maximum(lower, to_box)
        cell = np.floor(grid + 0.5).astype(int)
        in_grid = np.all((cell >= 0) & (cell < self.shape), axis=1)
        cell = np.clip(cell, 0, self.shape - 1)
        inside = in_grid & (self.distance[:, cell[..., 0], cell[..., 1], cell[..., 2]] == 0)
        return lower, inside

    def needle(self, tool, head_from_base, tip_length_mm):
        count = max(1, math.ceil(tip_length_mm / 2.0))
        start = tool[:3, 3]
        axis = tool[:3, 2]
        samples = start + np.linspace(0, tip_length_mm, count + 1)[:, None] * axis
        lower, inside = self.point_bounds(transform_points(head_from_base, samples))
        body, index = np.unravel_index(np.argmin(lower), lower.shape)
        # Prefer a true interior witness over a nearby zero lower bound.
        hits = np.argwhere(inside)
        if len(hits):
            tip_hits = hits[hits[:, 1] == count]
            body, index = (tip_hits if len(tip_hits) else hits)[0]
        lower_mm = float(lower.min() - NEEDLE_RADIUS_MM - tip_length_mm / (2 * count))
        return {"lower_mm": lower_mm, "structure": STRUCTURES[body], "part": "tip" if index == count else "shaft",
                "witness_base_mm": samples[index].tolist(), "tool_base_mm": start.tolist(),
                "tip_base_mm": samples[-1].tolist(), "tip_inside": [STRUCTURES[i] for i in np.flatnonzero(inside[:, -1])],
                "interior_witness": bool(len(hits)),
                "intersections": {name: {"tip": bool(inside[i, -1]), "shaft": bool(inside[i, :-1].any())}
                                  for i, name in enumerate(STRUCTURES)}}


def certify_sweep(positions, times, kinematics, clearance, head_from_base, margin_mm=CLEARANCE_MM):
    """Certify the piecewise-linear joint path over every closed time interval."""
    q = np.asarray(positions, dtype=float)
    times = np.asarray(times, dtype=float)
    if (q.ndim != 2 or q.shape[1] != len(JOINT_NAMES) or q.shape[0] < 2
            or times.shape != (len(q),) or not np.isfinite(q).all()
            or not np.isfinite(times).all() or np.any(np.diff(times) < 0)):
        raise ValueError("Malformed planned joint path")
    evaluated = {}
    certified = []

    def sample(segment, fraction):
        key = segment + fraction
        if key not in evaluated:
            joints = q[segment] + fraction * (q[segment + 1] - q[segment])
            frames = kinematics.transforms(joints)
            report = clearance.needle(frames["tool0"], head_from_base, kinematics.tip_length_mm)
            evaluated[key] = {**report, "joints_rad": joints.tolist(),
                              "segment": segment, "fraction": fraction,
                              "time_s": float(times[segment] + fraction * (times[segment + 1] - times[segment])),
                              "links_base_mm": {name: matrix.tolist() for name, matrix in frames.items()}}
        return evaluated[key]

    def cover(segment, left, right, depth, motion):
        a, b = sample(segment, left), sample(segment, right)
        closest = min((a, b), key=lambda item: item["lower_mm"])
        if closest["lower_mm"] < margin_mm:
            return closest, "interior" if closest["interior_witness"] else "clearance_below_margin"
        # Every intermediate joint state is at most half the interval away
        # from one endpoint. Summing downstream lever arms bounds material point travel.
        lower = closest["lower_mm"] - motion * (right - left) / 2.0
        if lower >= margin_mm:
            certified.append(lower)
            return None
        if depth >= 12 or len(evaluated) >= 2000:
            return closest, "interval_unresolved"
        mid = (left + right) / 2.0
        failure = cover(segment, left, mid, depth + 1, motion)
        return failure or cover(segment, mid, right, depth + 1, motion)

    failure = None
    for i, delta in enumerate(np.diff(q, axis=0)):
        failure = cover(i, 0.0, 1.0, 0, float(np.abs(delta) @ kinematics.reach_mm))
        if failure:
            break
    frames = [evaluated[key] for key in sorted(evaluated)]
    closest = failure[0] if failure else min(frames, key=lambda item: item["lower_mm"])
    return {"passed": failure is None, "reason": failure[1] if failure else "certified",
            "margin_mm": margin_mm, "certified_minimum_lower_mm": min(certified) if not failure else None,
            "interval_count": len(certified), "sample_count": len(frames), "closest": closest, "frames": frames,
            "joint_interpolation": "piecewise_linear", "joint_reach_bounds_mm": kinematics.reach_mm.tolist()}
