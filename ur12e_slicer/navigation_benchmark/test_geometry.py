"""Mathematical oracle tests: known scalar coordinates, angles and speed x time.

Run: python -m unittest discover -s ur12e_slicer/navigation_benchmark -p test_geometry.py -v
"""

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

try:
    from .core import (ObservationRejected, estimated_robot_goal, invert,
        matrix_from_pose, optical_tip_in_image, pose_errors, pose_from_matrix,
        rigid_registration, validate_observations)
    from .run_offline import (check_fixture, evaluate_case, make_fixture,
                              run_evaluation, sha256)
except ImportError:
    from core import (ObservationRejected, estimated_robot_goal, invert,
        matrix_from_pose, optical_tip_in_image, pose_errors, pose_from_matrix,
        rigid_registration, validate_observations)
    from run_offline import (check_fixture, evaluate_case, make_fixture,
                             run_evaluation, sha256)


def observed(matrix, stamp=20.0, valid=True):
    return {"matrix": np.asarray(matrix).tolist(), "timestamp_s": stamp,
            "valid": valid, "clock_domain": "test"}


class GeometryOracles(unittest.TestCase):
    def test_quaternion_z_90_known_matrix(self):
        q = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
        expected = [[0, -1, 0, 10], [1, 0, 0, -20],
                    [0, 0, 1, 30], [0, 0, 0, 1]]
        np.testing.assert_allclose(matrix_from_pose([10, -20, 30], q), expected, atol=1e-12)

    def test_matrix_to_quaternion_half_turn(self):
        transform = np.diag([1., -1., -1., 1.])
        position, quaternion = pose_from_matrix(transform)
        np.testing.assert_allclose(position, [0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(np.abs(quaternion), [1, 0, 0, 0], atol=1e-12)

    def test_inverse_known_rotation_translation(self):
        transform = [[0, -1, 0, 10], [1, 0, 0, 20],
                     [0, 0, 1, 30], [0, 0, 0, 1]]
        expected = [[0, 1, 0, -20], [-1, 0, 0, 10],
                    [0, 0, 1, -30], [0, 0, 0, 1]]
        np.testing.assert_allclose(invert(transform), expected, atol=1e-12)

    def test_rigid_registration_known_point_coordinates(self):
        source = [[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [2, 3, 4]]
        # Hand-written H = (10-y, 20+x, 30+z), no chain function used.
        measured = [[10, 20, 30], [10, 21, 30], [8, 20, 30],
                    [10, 20, 33], [7, 22, 34]]
        transform, fre = rigid_registration(source, measured)
        expected = [[0, -1, 0, 10], [1, 0, 0, 20],
                    [0, 0, 1, 30], [0, 0, 0, 1]]
        np.testing.assert_allclose(transform, expected, atol=1e-12)
        self.assertLess(fre, 1e-12)

    def test_rigid_fit_does_not_absorb_scale(self):
        source = np.asarray([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float)
        transform, fre = rigid_registration(source, source*2)
        self.assertAlmostEqual(np.linalg.det(transform[:3, :3]), 1.0, places=12)
        self.assertGreater(fre, 0.1)

    def test_collinear_points_rejected(self):
        with self.assertRaises(ValueError):
            rigid_registration([[0, 0, 0], [1, 0, 0], [2, 0, 0]],
                               [[0, 0, 0], [0, 1, 0], [0, 2, 0]])

    def test_goal_chain_known_integer_coordinates(self):
        calibration = {
            "T_B_C": [[0, -1, 0, 10], [1, 0, 0, 20], [0, 0, 1, 30], [0, 0, 0, 1]],
            "T_H_I": [[1, 0, 0, 1], [0, 0, -1, 2], [0, 1, 0, 3], [0, 0, 0, 1]],
        }
        head = [[0, 0, 1, 4], [0, 1, 0, 5], [-1, 0, 0, 6], [0, 0, 0, 1]]
        target = np.eye(4)
        target[:3, 3] = [7, 8, 9]
        observations = {"now_s": 20, "head": observed(head)}
        actual = estimated_robot_goal(calibration, observations, target)
        # Scalar coordinate derivation: (7,8,9)->(8,-7,11)->(15,-2,-2)->(12,35,28).
        expected = [[0, 0, 1, 12], [0, 1, 0, 35], [-1, 0, 0, 28], [0, 0, 0, 1]]
        np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_optical_chain_known_tip_coordinates(self):
        calibration = {
            "T_H_I": [[1, 0, 0, 0], [0, 0, -1, 20], [0, 1, 0, 0], [0, 0, 0, 1]],
            "T_M_Tip": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 2], [0, 0, 0, 1]],
        }
        observations = {"now_s": 20,
            "head": observed([[0, -1, 0, 10], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
            "tool": observed([[0, 0, 1, -9], [1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 0, 1]])}
        expected = np.eye(4)
        expected[:3, 3] = [1, 2, 3]
        np.testing.assert_allclose(optical_tip_in_image(calibration, observations), expected, atol=1e-12)

    def test_axis_versus_roll_are_distinct(self):
        roll180 = np.diag([-1., -1., 1., 1.])
        errors = pose_errors(roll180, np.eye(4))
        self.assertEqual(errors["axis_error_deg"], 0)
        self.assertEqual(errors["orientation_error_deg"], 180)
        self.assertEqual(errors["position_error_mm"], 0)

    def test_needle_axis_tilt_90(self):
        tilt90 = [[1, 0, 0, 3], [0, 0, -1, 4], [0, 1, 0, 0], [0, 0, 0, 1]]
        errors = pose_errors(tilt90, np.eye(4))
        self.assertAlmostEqual(errors["position_error_mm"], 5)
        self.assertAlmostEqual(errors["axis_error_deg"], 90)
        self.assertAlmostEqual(errors["orientation_error_deg"], 90)

    def test_inputs_not_mutated(self):
        matrix = np.eye(4)
        original = matrix.copy()
        invert(matrix)
        pose_from_matrix(matrix)
        np.testing.assert_array_equal(matrix, original)


class ObservationGates(unittest.TestCase):
    def setUp(self):
        self.observations = {"now_s": 20,
            "head": observed(np.eye(4)), "tool": observed(np.eye(4))}

    def rejected(self, code):
        with self.assertRaises(ObservationRejected) as caught:
            validate_observations(self.observations)
        self.assertEqual(caught.exception.code, code)

    def test_missing_clock_rejected(self):
        del self.observations["now_s"]
        self.rejected("missing_clock")

    def test_equal_but_old_frames_rejected(self):
        for key in ("head", "tool"):
            self.observations[key]["timestamp_s"] = 19.75
        self.rejected("stale_frame")

    def test_fresh_but_skewed_frames_rejected(self):
        self.observations["tool"]["timestamp_s"] = 19.95
        self.rejected("frame_skew")

    def test_occlusion_rejected(self):
        self.observations["tool"]["valid"] = False
        self.rejected("tracking_invalid")

    def test_future_frame_rejected(self):
        self.observations["head"]["timestamp_s"] = 20.01
        self.rejected("future_frame")

    def test_different_clocks_rejected(self):
        self.observations["tool"]["clock_domain"] = "other_clock"
        self.rejected("clock_domain_mismatch")

    def test_nonrigid_matrix_rejected(self):
        self.observations["tool"]["matrix"][0][0] = 2
        self.rejected("invalid_transform")


class IndependentTruthEvaluation(unittest.TestCase):
    def setUp(self):
        self.fixture = make_fixture()
        self.target = self.fixture["truth"]["targets_I"][0]

    def test_seed_and_truth_not_mutated(self):
        before = sha256(self.fixture)
        self.assertEqual(before, sha256(make_fixture()))
        evaluate_case(self.fixture, "tool_calibration_bias", self.target)
        self.assertEqual(before, sha256(self.fixture))

    def test_fiducials_cannot_be_renamed_as_heldout(self):
        duplicate = copy.deepcopy(self.fixture)
        matrix = duplicate["truth"]["targets_I"][0]["matrix"]
        for index in range(3):
            matrix[index][3] = duplicate["truth"]["fiducials_I"][0][index]
        with self.assertRaises(ValueError):
            check_fixture(duplicate)

    def test_shared_registration_bias_has_zero_fre_but_nonzero_tre(self):
        row = evaluate_case(self.fixture, "registration_shared_bias", self.target)
        self.assertLess(row["registration_fre_mm"], 1e-10)
        self.assertAlmostEqual(row["true_position_error_mm"], 2.5, places=10)
        self.assertLess(row["display_position_error_mm"], 1e-10)

    def test_tool_bias_is_not_cancelled_in_truth(self):
        row = evaluate_case(self.fixture, "tool_calibration_bias", self.target)
        self.assertAlmostEqual(row["true_position_error_mm"], 2.5, places=10)
        self.assertAlmostEqual(row["true_axis_error_deg"], 3.0, places=10)
        self.assertAlmostEqual(row["true_orientation_error_deg"], 3.0, places=10)
        self.assertLess(row["display_position_error_mm"], 1e-10)

    def test_dynamic_error_matches_speed_times_delay(self):
        row = evaluate_case(self.fixture, "known_dynamic_delay", self.target)
        # 20 mm/s * 0.080 s = 1.600 mm in every rigid coordinate frame.
        self.assertAlmostEqual(row["true_position_error_mm"], 20*0.080, places=10)
        self.assertLess(row["display_position_error_mm"], 1e-10)

    def test_camera_offset_visible_to_independent_optical_display(self):
        row = evaluate_case(self.fixture, "camera_extrinsic_3mm", self.target)
        self.assertAlmostEqual(row["true_position_error_mm"], 3, places=10)
        self.assertAlmostEqual(row["display_position_error_mm"], 3, places=10)

    def test_invalid_rows_have_no_error_or_goal(self):
        for scenario in ("head_moved_after_plan", "head_occluded", "tool_occluded",
                         "stale_pair", "cross_object_skew"):
            row = evaluate_case(self.fixture, scenario, self.target)
            self.assertEqual(row["status"], "rejected")
            self.assertIsNone(row["goal_tip_B"])
            self.assertIsNone(row["true_position_error_mm"])
            self.assertIsNone(row["display_position_error_mm"])

    def test_reports_have_finite_values_and_separate_saved_truth(self):
        with tempfile.TemporaryDirectory(prefix="navigation_oracle_") as folder:
            run = run_evaluation(folder, self.fixture)
            self.assertTrue(run["all_checks_passed"])
            self.assertEqual(run["case_count"], 72)
            truth = json.loads((Path(folder)/"truth.json").read_text(encoding="utf-8"))
            self.assertEqual(truth["sha256"], sha256(truth["truth"]))
            raw = (Path(folder)/"run.json").read_text(encoding="utf-8")
            self.assertNotIn("NaN", raw)
            self.assertNotIn("Infinity", raw)
            self.assertIn('"true_position_error_mm": null', raw)
            self.assertTrue((Path(folder)/"report.html").is_file())


if __name__ == "__main__":
    unittest.main(verbosity=2)
