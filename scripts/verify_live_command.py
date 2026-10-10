import json
import numpy as np
import vtk

controls = slicer.classroomPractice
initial = controls.matrix().copy()
target = controls.target_position()
initial_distance = float(np.linalg.norm(initial[:3, 3] - target))
controls.move(-10)
moved = controls.matrix()
new_distance = float(np.linalg.norm(moved[:3, 3] - target))
assert abs(initial_distance - new_distance - 10) < 1e-6
tip_position = [0.0] * 3
slicer.util.getNode("SimulatedToolTip").GetNthControlPointPositionWorld(0, tip_position)
assert np.allclose(tip_position, moved[:3, 3])
line_start = [0.0] * 3
controls.distance.GetNthControlPointPositionWorld(0, line_start)
assert np.allclose(line_start, tip_position)
controls.move_sideways()
side_matrix = controls.matrix()
direction = target - side_matrix[:3, 3]
cos_angle = np.dot(direction / np.linalg.norm(direction), side_matrix[:3, 2])
assert cos_angle < 0.999
controls.reset()
assert np.allclose(controls.matrix(), initial)
frames = ROOT / "output" / "screenshots"
frames.mkdir(parents=True, exist_ok=True)
slicer.app.processEvents()
assert slicer.util.mainWindow().grab().save(str(frames / "MRI_practice.png"))
_result = {
    "slicer_version": slicer.app.applicationVersion,
    "initial_distance_mm": initial_distance,
    "after_move_distance_mm": new_distance,
    "tool_tip_world_follows_transform": True,
    "distance_line_follows_tip": True,
    "side_move_changes_direction": True,
    "reset_restores_initial_pose": True,
    "screenshot": str(frames / "MRI_practice.png"),
    "nodes": [{"name": n.GetName(), "type": n.GetClassName()} for n in slicer.util.getNodesByClass("vtkMRMLTransformableNode")],
}
(ROOT / "work" / "live_verification.json").write_text(json.dumps(_result, ensure_ascii=False, indent=2), encoding="utf-8")
