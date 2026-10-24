"""Protocol contracts usable without ROS; these tests do not claim robot motion."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ur12e_slicer_bridge.protocol import Target, atomic_json, pose_error, shifted_pose
from ur12e_slicer_bridge import protocol


class ProtocolTests(unittest.TestCase):
    def target(self, **changes):
        return {"schema_version": 1, "request_id": "test-1", "command": "plan_execute",
                "frame_id": "base", "position_m": [0.45, -0.35, 0.45],
                "orientation_xyzw": [1.0, 0.0, 0.0, 0.0], "timestamp": 1000.0, **changes}

    def test_tip_to_tool_conversion_respects_orientation(self):
        # A downward-pointing tool's flange must be ABOVE its commanded tip.
        target = Target.from_json(self.target(), 1001.0)
        flange = shifted_pose(target.position_m, target.orientation_xyzw, [0, 0, -0.12])
        self.assertAlmostEqual(flange[2], 0.57)
        recovered = shifted_pose(flange, target.orientation_xyzw, [0, 0, 0.12])
        self.assertAlmostEqual(recovered[2], target.position_m[2])

    def test_non_axis_aligned_tip_offset(self):
        root = math.sqrt(0.5)
        result = shifted_pose([0, 0, 0], [0, root, 0, root], [0, 0, 0.12])
        self.assertAlmostEqual(result[0], 0.12)
        self.assertAlmostEqual(result[2], 0.0)

    def test_quaternion_sign_does_not_change_orientation_error(self):
        distance, angle = pose_error([0, 0, 0], [1, 0, 0, 0], [0, 0, 0], [-1, 0, 0, 0])
        self.assertEqual(distance, 0.0)
        self.assertEqual(angle, 0.0)

    def test_reject_wrong_coordinate_contract_and_bad_numbers(self):
        for changes in ({"frame_id": "RAS"}, {"position_m": [True, 0, 0]},
                        {"position_m": [float("nan"), 0, 0]}, {"orientation_xyzw": [0, 0, 0, 0]},
                        {"schema_version": True}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Target.from_json(self.target(**changes), 1001.0)

    def test_reject_stale_and_future_commands(self):
        for timestamp in (900.0, 1012.0):
            with self.subTest(timestamp=timestamp), self.assertRaises(ValueError):
                Target.from_json(self.target(timestamp=timestamp), 1001.0)

    def test_execute_requires_plan_reference_and_matching_pose(self):
        with self.assertRaises(ValueError):
            Target.from_json(self.target(command="execute"), 1001.0)
        first = Target.from_json(self.target(command="plan"), 1001.0)
        second = Target.from_json(self.target(command="execute", request_id="exec-2", plan_request_id="test-1"), 1001.0)
        self.assertTrue(first.same_pose(second))
        different = Target.from_json(self.target(position_m=[0.46, -0.35, 0.45]), 1001.0)
        self.assertFalse(first.same_pose(different))

    def test_atomic_json_replacement_and_nan_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            atomic_json(path, {"status": "planning"})
            atomic_json(path, {"status": "succeeded", "source": "unit_test"})
            self.assertEqual(json.loads(path.read_text())["status"], "succeeded")
            with self.assertRaises(ValueError):
                atomic_json(path, {"position_error_m": float("nan")})
            self.assertEqual(json.loads(path.read_text())["status"], "succeeded")
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])

    def test_atomic_json_transient_permission_failure_retries_complete_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            path.write_text('{"sequence":1}', encoding="utf-8")
            replace = protocol.os.replace
            attempts = []

            def briefly_locked(temp, destination):
                # A failed replacement must leave the last valid state intact.
                self.assertEqual(json.loads(path.read_text())["sequence"], 1)
                self.assertEqual(json.loads(Path(temp).read_text())["sequence"], 2)
                attempts.append(temp)
                if len(attempts) < 3:
                    raise PermissionError(13, "injected shared-handle lock")
                replace(temp, destination)

            with patch.object(protocol.os, "replace", side_effect=briefly_locked), patch.object(protocol.time, "sleep") as sleep:
                atomic_json(path, {"sequence": 2})
            self.assertEqual(len(attempts), 3)
            self.assertEqual(len(set(attempts)), 1)
            self.assertEqual(sleep.call_count, 2)
            self.assertEqual(json.loads(path.read_text())["sequence"], 2)
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])

    def test_atomic_json_permanent_permission_failure_is_bounded(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            path.write_text('{"sequence":1}', encoding="utf-8")
            elapsed = [0.0]

            def advance(seconds):
                elapsed[0] += seconds

            with patch.object(protocol.os, "replace", side_effect=PermissionError(13, "injected persistent lock")) as replace, \
                    patch.object(protocol.time, "monotonic", side_effect=lambda: elapsed[0]), \
                    patch.object(protocol.time, "sleep", side_effect=advance):
                with self.assertRaises(PermissionError):
                    atomic_json(path, {"sequence": 2})
            self.assertGreater(replace.call_count, 1)
            self.assertLessEqual(replace.call_count, 10)
            self.assertLessEqual(elapsed[0], 0.08)
            self.assertEqual(json.loads(path.read_text())["sequence"], 1)
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])

    def test_atomic_json_unrelated_replace_error_does_not_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "state.json"
            path.write_text('{"sequence":1}', encoding="utf-8")
            with patch.object(protocol.os, "replace", side_effect=OSError(28, "injected storage failure")) as replace, \
                    patch.object(protocol.time, "sleep") as sleep:
                with self.assertRaises(OSError):
                    atomic_json(path, {"sequence": 2})
            replace.assert_called_once()
            sleep.assert_not_called()
            self.assertEqual(json.loads(path.read_text())["sequence"], 1)
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
