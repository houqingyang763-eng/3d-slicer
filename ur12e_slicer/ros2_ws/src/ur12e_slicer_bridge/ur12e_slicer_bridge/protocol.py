"""Protocol v1: SI units, UR base frame, xyzw quaternion, simulated needle tip."""
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


def vector(value, size, field):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{field} must contain {size} finite numbers")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{field} must contain {size} finite numbers")
    return tuple(float(x) for x in value)


def quaternion(value):
    q = vector(value, 4, "orientation_xyzw")
    maximum = max(abs(x) for x in q)
    if maximum < 1e-9:
        raise ValueError("orientation_xyzw must have nonzero norm")
    scaled = tuple(x / maximum for x in q)
    norm = math.sqrt(sum(x * x for x in scaled))
    return tuple(x / norm for x in scaled)


def rotate(q, v):
    x, y, z, w = quaternion(q)
    vx, vy, vz = vector(v, 3, "vector")
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def shifted_pose(position, orientation, offset):
    delta = rotate(orientation, offset)
    return [a + b for a, b in zip(position, delta)]


def pose_error(actual_position, actual_orientation, target_position, target_orientation):
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(actual_position, target_position)))
    dot = abs(sum(a * b for a, b in zip(quaternion(actual_orientation), quaternion(target_orientation))))
    return distance, 2 * math.acos(min(1.0, max(0.0, dot)))


@dataclass(frozen=True)
class Target:
    request_id: str
    command: str
    position_m: tuple
    orientation_xyzw: tuple
    timestamp: float
    plan_request_id: str = ""

    @classmethod
    def from_json(cls, data, now, max_age_s=60.0):
        if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
            raise ValueError("schema_version must be 1")
        request_id = data.get("request_id")
        if not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128:
            raise ValueError("request_id must be a nonempty string (<=128 characters)")
        command = data.get("command")
        if command not in ("plan", "execute", "plan_execute"):
            raise ValueError("command must be plan, execute, or plan_execute")
        if data.get("frame_id") != "base":
            raise ValueError("frame_id must be base; image/RAS coordinates must be converted first")
        timestamp = data.get("timestamp")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite Unix seconds")
        if now - timestamp > max_age_s or timestamp - now > 10:
            raise ValueError("command timestamp is stale or in the future; submit a fresh command")
        plan_id = data.get("plan_request_id", "")
        if not isinstance(plan_id, str) or (command == "execute" and not plan_id):
            raise ValueError("execute requires plan_request_id of a successfully cached plan")
        return cls(request_id, command, vector(data.get("position_m"), 3, "position_m"),
                   quaternion(data.get("orientation_xyzw")), float(timestamp), plan_id)

    def pose(self):
        return {"frame_id": "base", "position_m": list(self.position_m),
                "orientation_xyzw": list(self.orientation_xyzw)}

    def same_pose(self, other):
        distance, angle = pose_error(self.position_m, self.orientation_xyzw,
                                     other.position_m, other.orientation_xyzw)
        return distance < 1e-8 and angle < 1e-7


def atomic_json(path, data):
    """Replace complete JSON; briefly retry Windows shared-handle contention."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            handle.flush()
            os.fsync(handle.fileno())
        # Windows readers may briefly deny replacement through WSL's /mnt/c.
        # Keep the existing destination intact, retry only that sharing failure,
        # and stay below one feedback period (100 ms); freshness is unchanged.
        deadline = time.monotonic() + 0.08
        while True:
            try:
                os.replace(temp, path)
                break
            except PermissionError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise
                time.sleep(min(0.01, remaining))
    finally:
        if os.path.exists(temp):
            os.unlink(temp)
