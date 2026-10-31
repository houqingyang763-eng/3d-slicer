"""Start the real Slicer panel acceptance for the already verified URSim backend.

Execute through this task's slicer/task_client.py after the URSim-only wrapper
has checked the local container and ROS configuration. This entry does not start
services. It sends the panel's usual nearby target: base Z -25 mm, yaw +3 deg.
"""
import importlib
import sys

sys.path.insert(0, str(PROJECT / "validation"))
import slicer_acceptance

if getattr(slicer, "ur12eAcceptance", None) and slicer.ur12eAcceptance.timer.isActive():
    raise RuntimeError("An acceptance run is already active")
importlib.reload(slicer_acceptance)
slicer_acceptance.prepare(expected_source="ros2_ursim")
_result = slicer.ur12eAcceptance.start()
