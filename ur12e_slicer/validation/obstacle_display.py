"""Display boxes confirmed by MoveIt GetPlanningScene in the real Slicer GUI."""
import json
import math
from pathlib import Path
import time

import numpy as np
import slicer
import vtk

from ur12e_panel import PROJECT, pose_matrix, vtk_matrix, model


def sync(capture_relative=None, align_target=False, save_scene_relative=None):
    path = PROJECT / "runtime" / "obstacle_scene.json"
    readback = json.loads(path.read_text(encoding="utf-8"))
    if readback.get("source") != "moveit_get_planning_scene" or readback.get("frame_id") != "base":
        raise RuntimeError("Display requires actual MoveIt planning-scene readback in base")
    if abs(time.time() - readback["timestamp"]) > 60:
        raise RuntimeError("Planning-scene readback is stale; inspect the live scene first")
    panel = slicer.ur12eSimulation
    panel.poll()
    if align_target and panel.state and panel.state.get("target_pose"):
        matrix = pose_matrix(panel.state["target_pose"])
        rotation = matrix[:3, :3]
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
        if abs(math.cos(pitch)) > 1e-6:
            roll, yaw = math.atan2(rotation[2, 1], rotation[2, 2]), math.atan2(rotation[1, 0], rotation[0, 0])
        else:
            roll, yaw = 0.0, math.atan2(-rotation[0, 1], rotation[1, 1])
        panel.updating_target = True
        try:
            for spin, value in zip(panel.position, matrix[:3, 3]):
                spin.setValue(float(value))
            for spin, value in zip(panel.angles, np.rad2deg([roll, pitch, yaw])):
                spin.setValue(float(value))
        finally:
            panel.updating_target = False
        panel.update_target()
    previous = getattr(slicer, "ur12eObstacleNodes", [])
    for node in previous:
        if node.GetScene():
            slicer.mrmlScene.RemoveNode(node)
    slicer.ur12eObstacleNodes = []
    checks = []
    for item in readback["objects"]:
        for index, box in enumerate(item["primitives"]):
            if box.get("type") != "box":
                raise RuntimeError("Only confirmed box primitives are supported")
            source = vtk.vtkCubeSource()
            source.SetXLength(box["dimensions_m"][0]*1000)
            source.SetYLength(box["dimensions_m"][1]*1000)
            source.SetZLength(box["dimensions_m"][2]*1000)
            source.Update()
            node = model(item["id"]+"_"+str(index), source.GetOutput(), [.90, .25, .16], .55)
            transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", item["id"]+"_base_pose")
            expected = pose_matrix(box["pose_base"])
            transform.SetMatrixTransformToParent(vtk_matrix(expected))
            node.SetAndObserveTransformNodeID(transform.GetID())
            node.SetAttribute("UR12e.Source", "moveit_get_planning_scene")
            node.SetAttribute("UR12e.ObjectID", item["id"])
            node.SetAttribute("UR12e.ReadbackTimestamp", str(readback["timestamp"]))
            slicer.ur12eObstacleNodes.extend([node, transform])
            actual = vtk.vtkMatrix4x4()
            transform.GetMatrixTransformToParent(actual)
            difference = max(abs(actual.GetElement(i, j)-expected[i, j]) for i in range(4) for j in range(4))
            lengths = np.array(source.GetOutput().GetBounds()).reshape(3, 2)
            size_difference = float(np.max(np.abs(np.diff(lengths, axis=1).ravel()-np.array(box["dimensions_m"])*1000)))
            checks.append({"object_id": item["id"], "dimensions_m": box["dimensions_m"],
                           "pose_base": box["pose_base"], "transform_max_difference": float(difference),
                           "dimensions_mm_max_difference": size_difference,
                           "display_matches_moveit_readback": difference < 1e-8 and size_difference < 1e-5})
    result = {"source": readback["source"], "readback_timestamp": readback["timestamp"],
              "object_count": len(readback["objects"]), "checks": checks,
              "all_displayed_boxes_match_readback": all(c["display_matches_moveit_readback"] for c in checks)}
    if save_scene_relative:
        destination = PROJECT / save_scene_relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        result["scene_file"] = str(destination)
        result["scene_saved"] = bool(slicer.util.saveScene(str(destination)))
    if capture_relative:
        destination = PROJECT / capture_relative
        result["screenshot"] = slicer.ur12eSimulation.capture(destination)
        destination.with_suffix(".json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result
