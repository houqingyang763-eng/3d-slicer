#!/usr/bin/env python3
"""Real mock-robot gate: lateral box, blocked goal, recovery; no detour claim.

The backend must already be running and this script must have the exclusive
target-sending window. Run under WSL on the same clock as the ROS backend.
--slicer also captures the real GUI through its existing task-only file queue.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

import rclpy

from scene_tool import SceneTool, PREFIX, atomic_json
from validate_ros_runtime import Validation, joint_delta

PROJECT = Path(__file__).resolve().parents[1]
SIDE = PREFIX + "side_box"
BLOCK = PREFIX + "goal_blocker"


def slicer_capture(relative, save_scene=False):
    sys.path.insert(0, str(PROJECT / "slicer"))
    from task_client import send
    code = ("import sys, importlib\n"
            "sys.path.insert(0, str(PROJECT / 'validation'))\n"
            "import obstacle_display\nimportlib.reload(obstacle_display)\n"
            f"_result = obstacle_display.sync({str(relative)!r}, align_target=True, save_scene_relative={str(Path(relative).with_suffix('.mrb'))!r} if {save_scene!r} else None)\n")
    result = send(code, timeout=30)
    return result.get("result", result)


class ObstacleValidation(Validation):
    def __init__(self, args):
        super().__init__(args)
        self.scene = SceneTool()
        self.summary.update({"mode": "real_moveit_obstacle_scene", "slicer_captures": [],
                             "scope": "Lateral obstacle with real planning and execution; collision-blocked goal rejection and recovery. This does not establish a detour or continuous collision-proof certificate."})

    def read_state(self):
        # DrvFs can expose a very short ENOENT/EACCES window while Windows and
        # WSL replace/read the shared file. Reopen briefly, then retain every
        # original freshness/source/numeric check from the backend validator.
        deadline = time.monotonic()+0.25
        while True:
            try:
                return super().read_state()
            except (FileNotFoundError, PermissionError):
                if time.monotonic() >= deadline:
                    raise
                self.summary["short_shared_file_read_retries"] = self.summary.get("short_shared_file_read_retries", 0)+1
                time.sleep(.01)

    def scene_save(self, label):
        state = self.scene.snapshot(label)
        atomic_json(self.output / (label+"_scene.json"), state)
        assert "slicer_simplified_needle" in state["attached_object_ids"], "Needle collision geometry disappeared from actual MoveIt scene"
        return state

    def capture(self, label, save_scene=False):
        if self.args.slicer:
            relative = (self.output / (label+".png")).relative_to(PROJECT).as_posix()
            result = slicer_capture(relative, save_scene=save_scene)
            self.summary["slicer_captures"].append(result)
            assert result.get("all_displayed_boxes_match_readback"), "Slicer boxes differ from actual planning-scene readback"
            if save_scene:
                assert result.get("scene_saved"), "Slicer did not save the actual obstacle scene"

    def plan(self, pose, label, before):
        request = self.submit("plan", pose, label)
        state, statuses, observations = self.wait_terminal(request)
        assert state["status"] == "planned", f"{label}: {state['message']}"
        assert state["plan"].get("moveit_error_code") == 1, "Real planning service did not succeed"
        state = self.wait_preview(request["request_id"], observations)
        state = self.settle(request["request_id"], observations=observations)
        movement = max(joint_delta(before, s) for s in observations+[state])
        assert movement < 1e-5 and state["execution"]["status"] == "none", "Plan-only unexpectedly moved the mock robot"
        self.save_case(label, request, state, max_joint_change_rad=movement, passed=True)
        return request, state

    def validate_trajectory(self, planned):
        trajectory = planned["plan"]["joint_trajectory"]
        names, points = trajectory["joint_names"], trajectory["points"]
        samples = []
        previous = points[0]["positions_rad"]
        for index, point in enumerate(points):
            current = point["positions_rad"]
            count = max(1, math.ceil(max(abs(a-b) for a, b in zip(previous, current))/self.args.joint_sample_step))
            for step in range(1, count+1):
                alpha = step/count
                positions = [a+(b-a)*alpha for a, b in zip(previous, current)]
                valid = self.scene.validity(names, positions)
                samples.append({"planned_point_index": index, "interpolation_fraction": alpha,
                                "joint_positions_rad": positions, **valid})
                if not valid["valid"]:
                    atomic_json(self.output / "trajectory_collision_samples.json", samples)
                    raise AssertionError("Actual GetStateValidity rejected a sampled trajectory state with obstacle present")
            previous = current
        atomic_json(self.output / "trajectory_collision_samples.json", samples)
        return {"sample_count": len(samples), "all_samples_valid": all(s["valid"] for s in samples),
                "maximum_joint_step_rad": self.args.joint_sample_step,
                "service": "/check_state_validity", "continuous_certificate": False}

    def run(self):
        initial = self.wait_ready()
        atomic_json(self.output / "initial_state.json", initial)
        # Remove only this utility's two exact IDs, preserving every other scene object.
        self.scene.remove(BLOCK)
        self.scene.remove(SIDE)
        self.scene_save("00_clear")
        target = {"position_m": [x+d for x, d in zip(initial["tcp_pose"]["position_m"], (0, 0, -0.025))],
                  "orientation_xyzw": initial["tcp_pose"]["orientation_xyzw"]}
        _, baseline = self.plan(target, "01_unobstructed_plan", initial)

        # Pick a modest lateral scene which leaves the observed start and already
        # solved endpoint valid; the new trajectory is still planned with the box.
        candidates = [(0.15, 0, 0.04), (-0.15, 0, 0.04), (0, 0.15, 0.04), (0, -0.15, 0.04), (0.20, 0, -0.05)]
        attempts = []
        endpoint = baseline["plan"]["joint_trajectory"]["points"][-1]["positions_rad"]
        for delta in candidates:
            center = [x+d for x, d in zip(initial["tcp_pose"]["position_m"], delta)]
            self.scene.add_box(SIDE, center, [0.06, 0.06, 0.12])
            start_valid = self.scene.validity(initial["joint_names"], initial["joint_positions_rad"])
            goal_valid = self.scene.validity(baseline["plan"]["joint_trajectory"]["joint_names"], endpoint)
            attempts.append({"center_m": center, "start_validity": start_valid, "baseline_goal_validity": goal_valid})
            if start_valid["valid"] and goal_valid["valid"]:
                break
        else:
            raise AssertionError("No candidate lateral box left both start and endpoint valid")
        self.summary["side_box_candidates"] = attempts
        self.summary["side_box_scene"] = self.scene_save("02_side_box")
        plan_request, planned = self.plan(target, "02_with_side_box_plan", initial)
        self.summary["trajectory_validity"] = self.validate_trajectory(planned)
        self.capture("02_with_side_box_plan")
        execute = self.submit("execute", target, "03_with_side_box_execute", plan_request["request_id"])
        final, statuses, observed = self.wait_terminal(execute)
        final = self.check_execution("03_with_side_box_execute", execute, planned, final, statuses, observed)
        self.scene_save("03_side_box_executed")
        self.capture("03_with_side_box_executed", save_scene=True)

        # Prove the second goal is solvable before placing a box around its tip.
        blocked_target = {"position_m": [x+d for x, d in zip(final["tcp_pose"]["position_m"], (0, 0, -0.050))],
                          "orientation_xyzw": final["tcp_pose"]["orientation_xyzw"]}
        _, would_reach = self.plan(blocked_target, "04_before_goal_blocked", final)
        self.scene.add_box(BLOCK, blocked_target["position_m"], [0.04, 0.04, 0.04])
        self.summary["blocked_scene"] = self.scene_save("05_goal_blocked")
        still_valid = self.scene.validity(final["joint_names"], final["joint_positions_rad"])
        goal_positions = would_reach["plan"]["joint_trajectory"]["points"][-1]["positions_rad"]
        goal_invalid = self.scene.validity(would_reach["plan"]["joint_trajectory"]["joint_names"], goal_positions)
        self.summary["blocked_state_validity"] = {"current_state": still_valid, "previously_solved_goal": goal_invalid}
        assert still_valid["valid"], "Blocked-goal box already collides with the unchanged current robot"
        assert not goal_invalid["valid"], "Actual MoveIt state checker did not detect the solved goal in the box"
        assert any(BLOCK in (c["body_1"], c["body_2"]) for c in goal_invalid["contacts"]), "Collision result did not name the real world box"
        request = self.submit("plan_execute", blocked_target, "05_blocked_goal_rejected")
        rejected, statuses, observed = self.wait_terminal(request)
        rejected = self.settle(request["request_id"], observations=observed)
        movement = max(joint_delta(final, state) for state in observed+[rejected])
        passed = (rejected["status"] == "error" and rejected["plan"].get("moveit_error_code") not in (None, 1)
                  and rejected["execution"]["status"] == "none" and movement < 1e-5)
        self.save_case("05_blocked_goal_rejected", request, rejected,
                       observed_statuses=statuses, max_joint_change_rad=movement, passed=passed)
        assert passed, "Collision-blocked goal did not fail planning without moving"
        self.capture("05_blocked_goal_rejected", save_scene=True)
        self.scene.remove(BLOCK)
        self.scene_save("06_blocker_removed")
        self.plan(blocked_target, "06_goal_planning_recovered", final)
        self.capture("06_goal_planning_recovered", save_scene=True)
        self.summary["passed"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=PROJECT / "runtime")
    parser.add_argument("--output", type=Path, default=PROJECT / "output" / "obstacle_validation")
    parser.add_argument("--slicer", action="store_true")
    parser.add_argument("--ready-timeout", type=float, default=60)
    parser.add_argument("--step-timeout", type=float, default=150)
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance", type=float, default=0.03)
    parser.add_argument("--max-feedback-age", type=float, default=1.0)
    parser.add_argument("--joint-sample-step", type=float, default=0.03)
    parser.add_argument("--keep-side-box", action="store_true", help="Retain the side obstacle after successful validation; otherwise remove this utility's boxes")
    args = parser.parse_args()
    if not 0 < args.joint_sample_step <= 0.05:
        parser.error("joint-sample-step must be within (0, 0.05] rad")
    rclpy.init()
    gate = ObstacleValidation(args)
    lock = args.runtime / "validation.lock"
    owns_lock = False
    try:
        with lock.open("x", encoding="utf-8") as handle:
            json.dump({"pid": os.getpid(), "run_id": gate.run_id, "created_at": time.time(), "scope": "obstacle"}, handle)
        owns_lock = True
        gate.run()
    except Exception as exc:
        gate.summary.update({"error": str(exc), "traceback": traceback.format_exc()})
        print("FAIL: "+str(exc), flush=True)
    finally:
        if owns_lock:
            try:
                # The goal blocker is never left around the target after a failure.
                gate.scene.remove(BLOCK)
                if not args.keep_side_box or not gate.summary["passed"]:
                    gate.scene.remove(SIDE)
                gate.scene_save("final_scene")
                if args.slicer:
                    gate.capture("07_cleanup")
            except Exception:
                gate.summary["cleanup_error"] = traceback.format_exc()
                gate.summary["passed"] = False
            lock.unlink()
        gate.finish()
        gate.scene.close()
        rclpy.shutdown()
    return 0 if gate.summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
