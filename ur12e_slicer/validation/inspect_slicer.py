"""Read this task's actual Slicer UI state without sending targets."""
import time
p = slicer.ur12eSimulation
_result = {"slicer_pid": p.session_info["pid"], "status_text": p.status_label.text,
           "error_text": p.error_label.text, "model_text": p.model_label.text,
           "model_links": sorted(p.mesh_transforms), "state": p.state,
           "target": p.current_target(), "recording": str(p.recording) if p.recording else None,
           "timestamp": time.time(), "queue": str(PROJECT / "runtime" / "slicer_bridge")}
