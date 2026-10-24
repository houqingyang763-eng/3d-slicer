"""Load acceptance runner without sending a robot command."""
import sys
import importlib
sys.path.insert(0, str(PROJECT / "validation"))
import slicer_acceptance
if getattr(slicer, "ur12eAcceptance", None) and slicer.ur12eAcceptance.timer.isActive():
    raise RuntimeError("An acceptance run is already active")
importlib.reload(slicer_acceptance)
_result = slicer_acceptance.prepare()
