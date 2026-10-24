"""Rigid navigation geometry. T_A_B maps B coordinates into A; lengths are mm.

No ROS, device or wall-clock dependency. Observation timestamps and now_s must
be in the same explicit clock domain. This is an engineering simulation API,
not a medical device, hardware accuracy measurement or ASTM conformity test.
"""

import math
import numpy as np


class ObservationRejected(ValueError):
    def __init__(self, code, detail):
        self.code = code
        self.detail = detail
        ValueError.__init__(self, "{}: {}".format(code, detail))


def rigid_matrix(value):
    """Copy and validate a right-handed rigid homogeneous transform."""
    result = np.array(value, dtype=float, copy=True)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError("Transform must be a finite 4 x 4 matrix")
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError("Invalid homogeneous bottom row")
    rotation = result[:3, :3]
    if (not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation) - 1.0) > 1e-6):
        raise ValueError("Rotation must be orthogonal with determinant +1")
    return result


def matrix_from_pose(position_mm, quaternion_xyzw):
    position = np.asarray(position_mm, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion))):
        raise ValueError("Expected finite xyz[3] and xyzw[4]")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("Zero quaternion")
    x, y, z, w = quaternion / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    result[:3, 3] = position
    return rigid_matrix(result)


def pose_from_matrix(value):
    transform = rigid_matrix(value)
    r = transform[:3, :3]
    trace = float(np.trace(r))
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
             (r[1, 0]-r[0, 1])/s, s/4]
    else:
        index = int(np.argmax(np.diag(r)))
        if index == 0:
            s = 2 * math.sqrt(max(0.0, 1+r[0, 0]-r[1, 1]-r[2, 2]))
            q = [s/4, (r[0, 1]+r[1, 0])/s, (r[0, 2]+r[2, 0])/s,
                 (r[2, 1]-r[1, 2])/s]
        elif index == 1:
            s = 2 * math.sqrt(max(0.0, 1+r[1, 1]-r[0, 0]-r[2, 2]))
            q = [(r[0, 1]+r[1, 0])/s, s/4, (r[1, 2]+r[2, 1])/s,
                 (r[0, 2]-r[2, 0])/s]
        else:
            s = 2 * math.sqrt(max(0.0, 1+r[2, 2]-r[0, 0]-r[1, 1]))
            q = [(r[0, 2]+r[2, 0])/s, (r[1, 2]+r[2, 1])/s, s/4,
                 (r[1, 0]-r[0, 1])/s]
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return transform[:3, 3].tolist(), q.tolist()


def invert(value):
    transform = rigid_matrix(value)
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3].dot(transform[:3, 3])
    return result


def transform_points(transform, points):
    transform = rigid_matrix(transform)
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError("Expected finite N x 3 points")
    return points.dot(transform[:3, :3].T) + transform[:3, 3]


def rigid_registration(points_I, points_H):
    """Fit T_H_I by proper rigid least squares (no scale, no reflection).

    Return matrix and fiducial residual RMSE. This FRE is NOT held-out TRE.
    At least three non-collinear source and measured points are required.
    """
    source = np.asarray(points_I, dtype=float)
    measured = np.asarray(points_H, dtype=float)
    if (source.ndim != 2 or source.shape[1] != 3 or measured.shape != source.shape
            or len(source) < 3 or not np.all(np.isfinite(source))
            or not np.all(np.isfinite(measured))):
        raise ValueError("Expected matching finite N x 3 fiducials, N >= 3")
    source_center = np.mean(source, axis=0)
    measured_center = np.mean(measured, axis=0)
    a = source - source_center
    b = measured - measured_center
    if np.linalg.matrix_rank(a) < 2 or np.linalg.matrix_rank(b) < 2:
        raise ValueError("Collinear fiducials cannot determine a rigid pose")
    u, unused, vt = np.linalg.svd(a.T.dot(b))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T.dot(u.T))
    rotation = vt.T.dot(correction).dot(u.T)
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = measured_center - rotation.dot(source_center)
    residuals = transform_points(result, source) - measured
    fre = math.sqrt(float(np.mean(np.sum(residuals * residuals, axis=1))))
    return rigid_matrix(result), fre


def validate_observations(observations, required=("head", "tool"), now_s=None,
                          max_age_s=0.100, max_skew_s=0.030):
    """Reject missing/invalid/stale/future/skewed measurements; never extrapolate.

    Caller must supply now_s or observations['now_s']. Equal timestamps alone
    are insufficient: a captured pair can be equally old. Policies are explicit
    assumed engineering limits, not claimed measured device specifications.
    """
    if now_s is None:
        now_s = observations.get("now_s")
    try:
        now_s = float(now_s)
    except (TypeError, ValueError):
        raise ObservationRejected("missing_clock", "Explicit finite current time is required")
    if not math.isfinite(now_s):
        raise ObservationRejected("missing_clock", "Explicit current time is required")
    if (not math.isfinite(max_age_s) or not math.isfinite(max_skew_s)
            or max_age_s < 0 or max_skew_s < 0):
        raise ValueError("Observation time limits must be nonnegative")
    stamps = []
    validated = {}
    clock_domains = []
    for name in required:
        observation = observations.get(name)
        if observation is None:
            raise ObservationRejected("missing_observation", name)
        if observation.get("valid") is not True:
            raise ObservationRejected("tracking_invalid", name)
        stamp = observation.get("timestamp_s")
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            raise ObservationRejected("missing_timestamp", name)
        if not math.isfinite(stamp):
            raise ObservationRejected("missing_timestamp", name)
        age = float(now_s) - float(stamp)
        if age < -1e-9:
            raise ObservationRejected("future_frame", name)
        if age > max_age_s + 1e-9:
            raise ObservationRejected("stale_frame", "{} age={:.6f}s".format(name, age))
        try:
            validated[name] = rigid_matrix(observation["matrix"])
        except (ValueError, KeyError, TypeError):
            raise ObservationRejected("invalid_transform", name)
        stamps.append(float(stamp))
        if "clock_domain" in observation:
            clock_domains.append(str(observation["clock_domain"]))
    if len(set(clock_domains)) > 1:
        raise ObservationRejected("clock_domain_mismatch", ",".join(clock_domains))
    if stamps and max(stamps) - min(stamps) > max_skew_s + 1e-9:
        raise ObservationRejected("frame_skew", "Pair is not sufficiently synchronized")
    return validated


def estimated_robot_goal(calibration, observations, target_I, **time_policy):
    """Return desired tip pose T_B_G, requiring valid current head tracking.

    A robot controlling marker/flange M must command T_B_G @ inverse(T_M_Tip).
    This function does not establish reachability, collision or hardware safety.
    """
    poses = validate_observations(observations, required=("head",), **time_policy)
    return (rigid_matrix(calibration["T_B_C"]).dot(poses["head"])
            .dot(rigid_matrix(calibration["T_H_I"])).dot(rigid_matrix(target_I)))


def optical_tip_in_image(calibration, observations, **time_policy):
    """Return optically estimated T_I_Tip, with both head and tool gates."""
    poses = validate_observations(observations, required=("head", "tool"), **time_policy)
    return (invert(calibration["T_H_I"]).dot(invert(poses["head"]))
            .dot(poses["tool"]).dot(rigid_matrix(calibration["T_M_Tip"])))


def pose_errors(actual, target):
    """Position, needle +Z axis and full SO(3) error; degrees, millimetres."""
    actual = rigid_matrix(actual)
    target = rigid_matrix(target)
    dot_axis = float(actual[:3, 2].dot(target[:3, 2]))
    rotation_cosine = (float(np.trace(target[:3, :3].T.dot(actual[:3, :3]))) - 1) / 2
    return {
        "position_error_mm": float(np.linalg.norm(actual[:3, 3] - target[:3, 3])),
        "axis_error_deg": math.degrees(math.acos(float(np.clip(dot_axis, -1, 1)))),
        "orientation_error_deg": math.degrees(math.acos(float(np.clip(rotation_cosine, -1, 1)))),
    }


def validate_execution_snapshot(planned_head_C, observations,
                                max_translation_mm=0.5, max_rotation_deg=0.5,
                                **time_policy):
    """Invalidate a pending command after observed head motion; do not auto-follow.

    These limits are simulated policy assumptions. A physical system requires
    a separate stop mechanism and assessed limits; this is a geometry gate.
    """
    poses = validate_observations(observations, required=("head", "tool"), **time_policy)
    movement = pose_errors(poses["head"], planned_head_C)
    if (movement["position_error_mm"] > max_translation_mm
            or movement["orientation_error_deg"] > max_rotation_deg):
        raise ObservationRejected("head_pose_changed", "Pending command must be replanned")
    return movement
