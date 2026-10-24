"""Simulation command gate; it is not a robot hardware safety function.

Only this gateway checks observation/context validity. The existing generic
file bridge does not enforce optical validity, so direct bridge commands can
bypass this demonstration. No stop-time claim is made.
"""
import hashlib
import json
import time
import numpy as np

from .core import (ObservationRejected, estimated_robot_goal, pose_errors,
                   rigid_matrix, validate_observations)


def calibration_digest(calibration):
    text = json.dumps({key: rigid_matrix(calibration[key]).tolist()
                       for key in ("T_B_C", "T_H_I", "T_M_Tip")},
                      sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


class NavigationGate:
    def __init__(self, max_age_s=0.100, max_skew_s=0.030,
                 head_motion_mm=0.5, head_motion_deg=0.5,
                 clock=time.time, clock_domain="unix_seconds"):
        self.policy = {"max_age_s": max_age_s, "max_skew_s": max_skew_s}
        self.head_motion_mm = head_motion_mm
        self.head_motion_deg = head_motion_deg
        self.clock = clock
        self.clock_domain = clock_domain

    def current_poses(self, observations):
        for name in ("head", "tool"):
            if observations.get(name, {}).get("clock_domain") != self.clock_domain:
                raise ObservationRejected("clock_domain_mismatch", name)
        # Receiver-owned clock: replaying a packet's own now_s cannot renew it.
        return validate_observations(observations, now_s=self.clock(), **self.policy)

    def plan_context(self, calibration, observations, target_I, version="v1"):
        poses = self.current_poses(observations)
        goal = estimated_robot_goal(calibration, observations, target_I,
                                    now_s=self.clock(), **self.policy)
        return {"calibration_digest": calibration_digest(calibration),
                "version": version, "head_matrix": poses["head"].tolist(),
                "target_I": rigid_matrix(target_I).tolist(),
                "goal_base_mm": goal.tolist()}

    def authorize_execute(self, context, calibration, observations, target_I, version="v1"):
        poses = self.current_poses(observations)
        if version != context["version"] or calibration_digest(calibration) != context["calibration_digest"]:
            raise ObservationRejected("calibration_changed", "Replan with current calibration")
        if not np.allclose(rigid_matrix(target_I), context["target_I"], atol=1e-9, rtol=0):
            raise ObservationRejected("target_changed", "Replan with current image goal")
        motion = pose_errors(poses["head"], context["head_matrix"])
        if (motion["position_error_mm"] > self.head_motion_mm or
                motion["orientation_error_deg"] > self.head_motion_deg):
            raise ObservationRejected("head_moved", "Cached plan invalidated by head motion")
        return rigid_matrix(context["goal_base_mm"])
