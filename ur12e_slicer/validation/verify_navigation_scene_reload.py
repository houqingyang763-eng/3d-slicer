"""Run in a separate Slicer process; reopen the saved MRB and verify geometry."""
import hashlib
import json
from pathlib import Path
import traceback

import numpy as np
import qt
import slicer
import vtk

PROJECT = Path(__file__).resolve().parents[1]
OUTPUT = PROJECT / "output" / "navigation_benchmark"


def verify():
    report = {"passed": False, "simulation_only": True}
    try:
        result = json.loads((OUTPUT / "latest_result.json").read_text(encoding="utf-8"))
        scene = OUTPUT / "latest_scene.mrb"
        assert slicer.util.loadScene(str(scene)), "MRB could not be reopened"
        volume = slicer.util.getNode("NAVBENCH_public_MR_head")
        parent = volume.GetParentTransformNode()
        matrix = vtk.vtkMatrix4x4()
        parent.GetMatrixTransformToWorld(matrix)
        T_B_I = np.array([[matrix.GetElement(i, j) for j in range(4)] for i in range(4)])
        assert np.allclose(T_B_I, result["head"]["T_base_image_mm"], atol=1e-8, rtol=0)
        target_B = np.asarray(result["goal_base_mm"])
        target_I = np.linalg.solve(T_B_I, target_B[:, 3])
        ras_to_ijk = vtk.vtkMatrix4x4()
        volume.GetRASToIJKMatrix(ras_to_ijk)
        ijk = [0.0]*4
        ras_to_ijk.MultiplyPoint(target_I.tolist(), ijk)
        image = slicer.util.arrayFromVolume(volume)
        i, j, k = np.rint(ijk[:3]).astype(int)
        assert 0 <= k < image.shape[0] and 0 <= j < image.shape[1] and 0 <= i < image.shape[2]
        intensity = float(image[k, j, i])
        assert intensity > 0, "Frozen target voxel is outside the nonzero MR signal"
        fids = slicer.util.getNode("NAVBENCH_synthetic_registration_fiducials_image")
        targets = slicer.util.getNode("NAVBENCH_synthetic_independent_checkpoints_image")
        report.update({"passed": True, "mrb_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
            "volume_dimensions_kji": list(image.shape), "T_base_image_preserved": True,
            "target_image_RAS_mm": target_I[:3].tolist(), "target_ijk": ijk[:3],
            "target_inside_volume": True, "target_voxel_intensity": intensity,
            "fiducial_count": fids.GetNumberOfControlPoints(),
            "heldout_count": targets.GetNumberOfControlPoints(),
            "slicer_version": slicer.app.applicationVersion,
            "boundary": "MR signal check only; synthetic target, no anatomical or physical accuracy claim"})
    except Exception:
        report["error"] = traceback.format_exc()
    (OUTPUT / "scene_reload_verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    slicer.app.exit(0 if report["passed"] else 1)


qt.QTimer.singleShot(1000, verify)
