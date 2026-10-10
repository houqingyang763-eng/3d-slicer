"""Run in a separate hidden Slicer process to check saved-scene persistence."""
from pathlib import Path
import json
import traceback

import slicer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
result = {"ok": False}
try:
    checks = []
    for filename, volume_name in [("CT_threshold_practice.mrb", "CTBrain"), ("MRI_navigation_practice.mrb", "MRHead")]:
        slicer.mrmlScene.Clear(0)
        assert slicer.util.loadScene(str(ROOT / "output" / "scenes" / filename))
        volume = slicer.util.getNode(volume_name)
        assert volume.GetImageData().GetNumberOfPoints() > 0
        check = {"scene": filename, "volume_dimensions": list(volume.GetImageData().GetDimensions())}
        if volume_name == "CTBrain":
            segmentation = slicer.util.getNode("ThresholdPractice_NotValidatedAnatomy")
            assert segmentation.GetSegmentation().GetNumberOfSegments() == 1
            check["segments"] = 1
            check["mesh_points"] = slicer.util.getNode("CT_threshold_surface_model").GetPolyData().GetNumberOfPoints()
            assert check["mesh_points"] > 0
        else:
            transform = slicer.util.getNode("SimulatedToolToImage")
            tool = slicer.util.getNode("SimulatedToolModel")
            tip = slicer.util.getNode("SimulatedToolTip")
            assert tool.GetTransformNodeID() == transform.GetID()
            assert tip.GetTransformNodeID() == transform.GetID()
            actual = slicer.util.arrayFromTransformMatrix(transform)
            initial = np.asarray(json.loads(transform.GetAttribute("PracticeInitialMatrix")))
            assert np.allclose(actual, initial)
            check["tool_transform_links_preserved"] = True
        checks.append(check)
    result.update(ok=True, checks=checks, slicer_version=slicer.app.applicationVersion)
except Exception:
    result["error"] = traceback.format_exc()
finally:
    (ROOT / "work" / "saved_scene_verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    slicer.app.exit(0 if result["ok"] else 1)
