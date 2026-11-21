"""Regression checks for workspace proposals and collision samples."""
import math
import unittest

import numpy as np

from ur12e_slicer_bridge.anatomy_geometry import pose_matrix_mm
from ur12e_slicer_bridge.head_routes import head_routes, joint_samples, route_waypoints


IDENTITY = (0.0, 0.0, 0.0, 1.0)


class ShaftProbe:
    def point_bounds(self, points):
        # The tip is above the obstacle, but the 20 mm shaft reaches it.
        inside = (np.abs(points[:, 0]) < 35) & (np.abs(points[:, 1]) < 35) & (points[:, 2] < -3)
        return None, inside


class HeadRoutesTests(unittest.TestCase):
    def routes(self, y=0, orientation=IDENTITY):
        start = pose_matrix_mm([-110, y, -20], IDENTITY)
        goal = np.array([110., y, 0])
        bounds = np.array([[-40, -40, -20], [40, 40, 40]])
        return head_routes(start, IDENTITY, goal, orientation, bounds, np.eye(4), ShaftProbe(), 20.)

    def test_shaft_intersection_prioritizes_detours(self):
        routes, blocked = self.routes()
        self.assertTrue(blocked)
        self.assertEqual(routes[-1][0], "straight")
        for _, tips in routes:
            np.testing.assert_allclose(tips[-1], [110, 0, 0])

    def test_clear_corridor_starts_with_straight_request(self):
        routes, blocked = self.routes(y=90)
        self.assertFalse(blocked)
        self.assertEqual(routes[0][0], "straight")

    def test_large_needle_tilt_leaves_routes_to_ompl(self):
        half = math.radians(52) / 2
        routes, _ = self.routes(orientation=(math.sin(half), 0, 0, math.cos(half)))
        self.assertEqual(routes, [])

    def test_tool_offset_is_applied_at_rotated_waypoints(self):
        tips = np.array([[25., 0, 45], [75., 20, 0]])
        angle = .25
        goal = (math.sin(angle), 0, 0, math.cos(angle))
        tool_poses = list(route_waypoints([0, 0, 0], IDENTITY, goal, tips, 120.))
        for (position_m, orientation), tip in zip(tool_poses, tips, strict=True):
            tool = pose_matrix_mm(position_m * 1000, orientation)
            np.testing.assert_allclose(tool[:3, 3] + 120 * tool[:3, 2], tip, atol=1e-12)
        np.testing.assert_allclose(tool_poses[-1][1], goal)

    def test_collision_samples_include_waypoints_and_segment_interiors(self):
        positions = np.zeros((3, 6))
        positions[1:, 0] = [.031, .009]
        samples = np.array(list(joint_samples(positions)))
        self.assertTrue((np.abs(np.diff(samples, axis=0)) <= .01 + 1e-12).all())
        for point in positions:
            self.assertTrue(np.any(np.all(np.isclose(samples, point), axis=1)))
