"""Focused checks for the whole-needle and between-waypoint clearance gates."""
import unittest

import numpy as np

from ur12e_slicer_bridge.anatomy_geometry import HeadClearance, certify_sweep


class LinearArm:
    """Synthetic geometry with a known 100 mm/rad bound on needle motion."""
    tip_length_mm = 30.0
    reach_mm = np.array([100.0, 0, 0, 0, 0, 0])

    def __init__(self, y):
        self.y = y

    def transforms(self, q):
        tool = np.eye(4)
        tool[:3, 3] = [100.0 * q[0], self.y, -15.0]
        return {"tool0": tool}


class AnatomyGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        origin = np.array([-48.0, -48.0, -48.0])
        centers = origin + 2.0 * np.moveaxis(np.indices((49, 49, 49)), 0, -1)
        cls.geometry = HeadClearance({"grid_origin_mm": origin, "grid_spacing_mm": 2.0,
                                      "brain_distance_mm": np.linalg.norm(centers, axis=-1),
                                      "skull_distance_mm": np.linalg.norm(centers - [-22, -32, 0], axis=-1)})

    def test_positive_clearance_outside_distance_grid(self):
        bounds, inside = self.geometry.point_bounds([[100.0, 0, 0]])
        self.assertGreater(bounds[0, 0], 95)
        self.assertFalse(inside.any())

    def test_shaft_detected_while_tip_is_outside(self):
        pose = LinearArm(0).transforms(np.zeros(6))["tool0"]
        witness = self.geometry.needle(pose, np.eye(4), 30)
        self.assertEqual(witness["tip_inside"], [])
        self.assertTrue(witness["intersections"]["brain"]["shaft"])
        self.assertEqual(witness["part"], "shaft")

    def test_clear_waypoints_cannot_hide_a_midsegment_intersection(self):
        positions = np.zeros((2, 6))
        positions[:, 0] = [-1.0, 1.0]
        arm = LinearArm(0)
        for q in positions:
            self.assertGreater(self.geometry.needle(arm.transforms(q)["tool0"], np.eye(4), 30)["lower_mm"], 5)
        report = certify_sweep(positions, [0, 1], arm, self.geometry, np.eye(4))
        self.assertFalse(report["passed"])
        self.assertEqual(report["reason"], "interior")
        self.assertEqual(report["closest"]["fraction"], .5)

    def test_full_safe_segment_has_certified_margin(self):
        positions = np.zeros((2, 6))
        positions[:, 0] = [-1.0, 1.0]
        report = certify_sweep(positions, [0, 1], LinearArm(25), self.geometry, np.eye(4))
        self.assertTrue(report["passed"])
        self.assertGreaterEqual(report["certified_minimum_lower_mm"], 5)
        self.assertGreater(report["interval_count"], 1)


if __name__ == "__main__":
    unittest.main()
