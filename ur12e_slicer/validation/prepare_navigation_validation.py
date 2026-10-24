"""Temporarily disable manual command input while a single runner owns it."""
import slicer
panel = getattr(slicer, "ur12eSimulation", None)
if panel is None:
    raise RuntimeError("The UR12e control panel is not initialized; validation did not acquire it.")
panel.dock.setEnabled(False)
panel.target_marker.SetLocked(True)
_result = {"manual_command_input_enabled": False, "feedback_polling_unchanged": True}
