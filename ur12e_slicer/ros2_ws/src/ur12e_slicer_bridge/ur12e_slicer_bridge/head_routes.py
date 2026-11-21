"""Short workspace routes around one confirmed head, in UR base coordinates."""
import math

import numpy as np

from .anatomy_geometry import pose_matrix_mm, transform_points


WHOLE_ARM_STEP_RAD = 0.01
ROUTE_PADDING_MM = 12.0
MAX_LOCAL_TILT_RAD = math.pi / 4


def interpolate_orientation(start, goal, fraction):
    start, goal = np.asarray(start), np.asarray(goal)
    dot = float(start @ goal)
    if dot < 0:
        goal, dot = -goal, -dot
    if dot > 0.9995:
        result = (1 - fraction) * start + fraction * goal
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    return (math.sin((1 - fraction) * angle) * start + math.sin(fraction * angle) * goal) / math.sin(angle)


def route_waypoints(start_tip, start_orientation, goal_orientation, tips, needle_mm):
    """Convert tip-space waypoints to tool0 poses, including orientation changes."""
    lengths = np.linalg.norm(np.diff(np.vstack([start_tip, tips]), axis=0), axis=1)
    total = float(lengths.sum())
    distance = 0.0
    for index, (tip, length) in enumerate(zip(tips, lengths, strict=True)):
        distance += length
        fraction = 1.0 if index == len(tips) - 1 else distance / total
        orientation = interpolate_orientation(start_orientation, goal_orientation, fraction)
        axis = pose_matrix_mm([0, 0, 0], orientation)[:3, 2]
        yield ((tip - needle_mm * axis) / 1000, orientation)


def head_routes(start_tool, start_orientation, goal_tip, goal_orientation, bounds_mm,
                head_from_base, clearance, needle_mm):
    """Rank Cartesian proposals. The label probe only changes search order."""
    goal_axis = pose_matrix_mm([0, 0, 0], goal_orientation)[:3, 2]
    start_tip = start_tool[:3, 3] + needle_mm * start_tool[:3, 2]
    difference = goal_tip - start_tip
    low, high = bounds_mm
    if (np.linalg.norm(difference) > 3 * max(high - low)
            or start_tool[:3, 2] @ goal_axis < math.cos(MAX_LOCAL_TILT_RAD)):
        return [], False

    # The Cartesian service linearly interpolates tool0 and slerps its orientation.
    # Probe that shaft sweep in the labelled head volume before calling MoveIt.
    goal_tool = goal_tip - needle_mm * goal_axis
    fractions = np.linspace(0, 1, 21)[1:-1]
    shafts = []
    for fraction in fractions:
        tool = start_tool[:3, 3] + fraction * (goal_tool - start_tool[:3, 3])
        rotation = interpolate_orientation(start_orientation, goal_orientation, fraction)
        axis = pose_matrix_mm([0, 0, 0], rotation)[:3, 2]
        shafts.append(tool + np.linspace(0, needle_mm, 7)[:, None] * axis)
    _, inside = clearance.point_bounds(transform_points(head_from_base, np.concatenate(shafts)))
    straight_blocked = bool(inside.any())

    detours = []
    horizontal = float(np.linalg.norm(difference[:2]))
    if horizontal > 1e-6:
        along = difference[:2] / horizontal
        side = np.array([-along[1], along[0]])
        corners = np.array([[x, y] for x in (low[0], high[0]) for y in (low[1], high[1])])
        longitudinal = (corners - start_tip[:2]) @ along
        limits = np.array([longitudinal.min(), longitudinal.max()]) + np.array([-1, 1]) * ROUTE_PADDING_MM
        edges = start_tip + np.clip(limits / horizontal, 0, 1)[:, None] * difference

        above = edges.copy()
        above[:, 2] = max(start_tip[2], goal_tip[2], high[2] + ROUTE_PADDING_MM)
        apex = above.mean(axis=0)
        apex[2] += 18.0
        detours.extend([("above_center", np.vstack([apex, goal_tip])), ("above", np.vstack([above, goal_tip]))])
        lateral = corners @ side
        for name, offset in (("left", lateral.max() + ROUTE_PADDING_MM),
                             ("right", lateral.min() - ROUTE_PADDING_MM)):
            bend = edges.copy()
            bend[:, :2] += (offset - bend[:, :2] @ side)[:, None] * side
            detours.append((name, np.vstack([bend, goal_tip])))

    detours.sort(key=lambda route: np.linalg.norm(np.diff(np.vstack([start_tip, route[1]]), axis=0), axis=1).sum())
    straight = [("straight", goal_tip[None, :])]
    return (detours + straight if straight_blocked else straight + detours), straight_blocked


def joint_samples(positions):
    """Yield complete trajectory states, with at most 0.01 rad per joint per step."""
    yield positions[0]
    for start, end in zip(positions, positions[1:]):
        delta = end - start
        count = max(1, math.ceil(np.max(np.abs(delta)) / WHOLE_ARM_STEP_RAD))
        for step in range(1, count + 1):
            yield start + (step / count) * delta


def path_quality(frames, goal):
    tips = np.asarray([frame["tip_base_mm"] for frame in frames])
    displacement = goal - tips[0]
    chord = float(np.linalg.norm(displacement))
    increments = np.diff(tips, axis=0)
    length = float(np.linalg.norm(increments, axis=1).sum())
    reverse = float(np.maximum(0, -(increments @ (displacement / chord))).sum()) if chord > 1e-6 else 0.0
    return {"tip_length_mm": length, "backtrack_mm": reverse, "direct_displacement_mm": chord,
            "score_mm": length + reverse, "short_forward_route": length <= 1.3 * chord + 2 and reverse <= 10}
