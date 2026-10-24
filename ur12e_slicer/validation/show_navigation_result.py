"""Display saved integration evidence in the running dedicated Slicer."""
from pathlib import Path
import json
import math
import sys

import numpy as np
import slicer

project = Path(PROJECT)
sys.path.insert(0, str(project / "slicer"))
import navigation_benchmark_scene
import importlib
importlib.reload(navigation_benchmark_scene)

result_path = project / "output" / "navigation_benchmark" / "latest_result.json"
result = json.loads(result_path.read_text(encoding="utf-8"))
panel = getattr(slicer, "ur12eSimulation", None)
if panel is not None:
    # Keep the existing console goal consistent with the final image goal;
    # updating a display never sends a target command.
    target = np.asarray(result["goal_base_mm"])
    r = target[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    panel.updating_target = True
    for widget, value in zip(panel.position, target[:3, 3]):
        widget.setDecimals(4)
        widget.setValue(float(value))
    for widget, value in zip(panel.angles, np.rad2deg([roll, pitch, yaw])):
        widget.setDecimals(4)
        widget.setValue(float(value))
    panel.updating_target = False
    panel.update_target()
    panel.dock.hide()
_result = navigation_benchmark_scene.run(result_path)
# Save proof that the target is also localized within the public MR volume,
# rather than only overlaying a surface mesh on the robot.
slice_info = navigation_benchmark_scene.show_slices(result)
slice_path = result_path.parent / "latest_mri_slices.png"
if not slicer.util.mainWindow().grab().save(str(slice_path)):
    raise RuntimeError("Could not save MRI slice evidence")
_result["artifacts"]["mri_screenshot"] = str(slice_path)
_result["mri_slice_check"] = slice_info
(result_path.parent / "scene_metadata.json").write_text(json.dumps(_result, ensure_ascii=False, indent=2), encoding="utf-8")
slicer.app.layoutManager().setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutOneUp3DView)
navigation_benchmark_scene.set_view(result, "head")
