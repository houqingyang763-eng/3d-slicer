"""Save this task's actual scene checkpoint, then request its normal exit."""
from pathlib import Path
import json
import os
import time

p = slicer.ur12eSimulation
folder = PROJECT / "output" / "paused_20261021"
folder.mkdir(parents=True, exist_ok=True)
scene_file = folder / "UR12e_paused_checkpoint.mrb"
saved = bool(slicer.util.saveScene(str(scene_file)))
screenshot = p.capture(folder / "slicer_paused_checkpoint.png")
report = {"checkpoint_only": True, "simulation_only": True,
          "actual_robot_connected": False, "slicer_closed_loop_acceptance_started": False,
          "saved_at_windows_clock": time.time(), "slicer_pid": os.getpid(),
          "scene_saved": saved, "scene_file": str(scene_file), "screenshot": screenshot,
          "displayed_robot_source": None if p.state is None else p.state.get("source"),
          "robot_display": "official_mesh" if p.mesh_transforms else "link_geometry_placeholder",
          "target_in_slicer_controls": p.current_target(), "last_frontend_request": p.last_request,
          "last_observed_ros_feedback": p.state,
          "feedback_age_s_cross_clock": None if p.state is None else time.time() - p.state.get("timestamp", 0),
          "note": "Pause checkpoint, not a completed Slicer goal-to-motion acceptance. Slicer target was not sent. Backend feedback timestamp uses WSL clock, which had approximately +0.6 s offset from Windows."}
(folder / "checkpoint.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
if hasattr(slicer, "ur12eAcceptance"):
    slicer.ur12eAcceptance.timer.stop()
    slicer.ur12eAcceptance.report["paused_without_start"] = True
    slicer.ur12eAcceptance.transition("paused_without_start")
if p.recording:
    p.toggle_recording()
p.session_info["closed_at"] = time.time()
p.session_info["heartbeat"] = 0.0
from ur12e_panel import atomic_json
atomic_json(PROJECT / "runtime" / "slicer_session.json", p.session_info)
_result = {"checkpoint": str(folder / "checkpoint.json"), "scene_saved": saved,
           "pid": os.getpid(), "normal_exit_requested": True}
qt.QTimer.singleShot(1000, lambda: slicer.app.exit(0))
