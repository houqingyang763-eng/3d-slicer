"""Release the temporary manual-input lock, leaving feedback intact."""
import slicer
panel = getattr(slicer, "ur12eSimulation", None)
if panel:
    panel.dock.setEnabled(True)
    panel.target_marker.SetLocked(False)
_result = {"manual_command_input_enabled": True}
