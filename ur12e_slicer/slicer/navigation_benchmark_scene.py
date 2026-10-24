"""Read-only Slicer display for the navigation benchmark simulation.

``run(path)`` reads one result JSON; ``build(result)`` accepts the same payload.
Only NAVBENCH_ nodes are replaced.  This module never sends a robot command,
starts a bridge, or synthesizes robot joint/TF feedback.  Existing UR12e models
remain driven by the existing panel.  A supplied, saved ROS mock-feedback
snapshot may be used to display a robot in a separate result viewer.
"""
from pathlib import Path
import datetime
import json
import math
import os
import sys
import traceback

import numpy as np
import qt
import slicer
import vtk


PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
PREFIX = "NAVBENCH_"
OWNER = "ur12e_slicer.navigation_benchmark"
DEFAULT_RESULT = PROJECT / "output" / "navigation_benchmark" / "latest_result.json"
_dock = None


def _matrix(value, label):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(label + " must be a finite 4 x 4 matrix in millimetres")
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(label + " has an invalid homogeneous bottom row")
    if not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(value[:3, :3]), 1, atol=1e-5):
        raise ValueError(label + " rotation must be proper and orthonormal")
    return value


def _vtk_matrix(matrix):
    result = vtk.vtkMatrix4x4()
    for row in range(4):
        for column in range(4):
            result.SetElement(row, column, float(matrix[row, column]))
    return result


def _own(node, name=None):
    if name:
        node.SetName(PREFIX + name)
    node.SetAttribute("NavigationBenchmark.Owner", OWNER)
    return node


def _new(classname, name):
    return _own(slicer.mrmlScene.AddNewNodeByClass(classname, PREFIX + name))


def _remove_owned_nodes():
    nodes = list(slicer.util.getNodesByClass("vtkMRMLNode"))
    for node in nodes:
        if node.GetAttribute("NavigationBenchmark.Owner") == OWNER or (node.GetName() or "").startswith(PREFIX):
            slicer.mrmlScene.RemoveNode(node)


def _model(name, data, color, opacity=1):
    node = _new("vtkMRMLModelNode", name)
    node.SetAndObservePolyData(data)
    node.CreateDefaultDisplayNodes()
    display = _own(node.GetDisplayNode(), name + "_display")
    display.SetScalarVisibility(False)
    display.SetColor(*color)
    display.SetOpacity(float(opacity))
    display.SetBackfaceCulling(False)
    return node


def _line(name, points, color, radius=1.0, opacity=1):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.all(np.isfinite(points)):
        return None
    vertices = vtk.vtkPoints()
    line = vtk.vtkPolyLine()
    line.GetPointIds().SetNumberOfIds(len(points))
    for index, point in enumerate(points):
        vertices.InsertNextPoint(*point)
        line.GetPointIds().SetId(index, index)
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(line)
    data = vtk.vtkPolyData()
    data.SetPoints(vertices)
    data.SetLines(cells)
    tube = vtk.vtkTubeFilter()
    tube.SetInputData(data)
    tube.SetRadius(float(radius))
    tube.SetNumberOfSides(12)
    tube.CappingOn()
    tube.Update()
    output = vtk.vtkPolyData()
    output.DeepCopy(tube.GetOutput())
    return _model(name, output, color, opacity)


def _transform(name, matrix):
    node = _new("vtkMRMLLinearTransformNode", name)
    node.SetMatrixTransformToParent(_vtk_matrix(matrix))
    return node


def _axes(name, matrix, length=35, radius=.9, opacity=1):
    transform = _transform(name + "_pose", matrix)
    for index, color in enumerate([(1, .25, .2), (.25, 1, .35), (.25, .55, 1)]):
        endpoint = [0, 0, 0]
        endpoint[index] = length
        node = _line(name + "_" + "XYZ"[index], [[0, 0, 0], endpoint], color, radius, opacity)
        node.SetAndObserveTransformNodeID(transform.GetID())
    return transform


def _points(name, points, color, label_prefix, size=4, text=1.2):
    values = np.asarray(points, dtype=float)
    if values.size == 0:
        return None
    values = values.reshape((-1, 3))
    if not np.all(np.isfinite(values)):
        raise ValueError(name + " points must be finite")
    node = _new("vtkMRMLMarkupsFiducialNode", name)
    node.CreateDefaultDisplayNodes()
    display = _own(node.GetDisplayNode(), name + "_display")
    for index, point in enumerate(values):
        node.AddControlPoint(vtk.vtkVector3d(*point), label_prefix + str(index + 1))
    node.SetLocked(True)
    display.SetSelectedColor(*color)
    display.SetColor(*color)
    display.SetGlyphScale(float(size))
    display.SetTextScale(float(text))
    return node


def _world(matrix, points):
    values = np.asarray(points, dtype=float).reshape((-1, 3))
    return (matrix @ np.c_[values, np.ones(len(values))].T).T[:, :3]


def _volume_surface(head, image_to_base):
    filename = Path(head.get("volume_path", REPO / "data" / "MR-head.nrrd"))
    if not filename.is_absolute():
        filename = REPO / filename
    if not filename.is_file():
        raise FileNotFoundError("Public demonstration MR volume not found: " + str(filename))
    volume = slicer.util.loadVolume(str(filename), {"name": PREFIX + "public_MR_head"})
    if volume is None:
        raise RuntimeError("Slicer could not load the demonstration volume")
    _own(volume, "public_MR_head")
    volume.SetAttribute("NavigationBenchmark.Source", "Public MR example; synthetic geometric points, no anatomical annotation")
    for node, suffix in [(volume.GetDisplayNode(), "display"), (volume.GetStorageNode(), "storage")]:
        if node:
            _own(node, "public_MR_head_" + suffix)
    transform = _transform("ImageRAS_to_robot_base_mm_SIMULATED_TRUTH", image_to_base)
    volume.SetAndObserveTransformNodeID(transform.GetID())
    slicer.util.setSliceViewerLayers(background=volume)
    values = slicer.util.arrayFromVolume(volume)
    positive = values[np.isfinite(values) & (values > 0)]
    if positive.size == 0:
        raise ValueError("MR example contains no positive finite intensities")
    threshold = float(head.get("surface_threshold", max(40.0, np.percentile(positive, 18))))
    blurred = vtk.vtkImageGaussianSmooth()
    blurred.SetInputData(volume.GetImageData())
    blurred.SetStandardDeviations(.85, .85, .85)
    blurred.SetRadiusFactors(1.5, 1.5, 1.5)
    binary = vtk.vtkImageThreshold()
    binary.SetInputConnection(blurred.GetOutputPort())
    binary.ThresholdByUpper(threshold)
    binary.SetInValue(1)
    binary.SetOutValue(0)
    binary.SetOutputScalarTypeToUnsignedChar()
    contour = vtk.vtkDiscreteMarchingCubes()
    contour.SetInputConnection(binary.GetOutputPort())
    contour.SetValue(0, 1)
    largest = vtk.vtkPolyDataConnectivityFilter()
    largest.SetInputConnection(contour.GetOutputPort())
    largest.SetExtractionModeToLargestRegion()
    reduced = vtk.vtkDecimatePro()
    reduced.SetInputConnection(largest.GetOutputPort())
    reduced.SetTargetReduction(.82)
    reduced.PreserveTopologyOn()
    reduced.SplittingOff()
    smooth = vtk.vtkWindowedSincPolyDataFilter()
    smooth.SetInputConnection(reduced.GetOutputPort())
    smooth.SetNumberOfIterations(18)
    smooth.BoundarySmoothingOff()
    smooth.FeatureEdgeSmoothingOff()
    smooth.SetPassBand(.08)
    smooth.NonManifoldSmoothingOn()
    smooth.NormalizeCoordinatesOn()
    ijk_to_ras = vtk.vtkMatrix4x4()
    volume.GetIJKToRASMatrix(ijk_to_ras)
    to_ras = vtk.vtkTransform()
    to_ras.SetMatrix(ijk_to_ras)
    converted = vtk.vtkTransformPolyDataFilter()
    converted.SetTransform(to_ras)
    converted.SetInputConnection(smooth.GetOutputPort())
    converted.Update()
    data = vtk.vtkPolyData()
    data.DeepCopy(converted.GetOutput())
    if data.GetNumberOfPoints() == 0:
        raise RuntimeError("No MR intensity surface was produced")
    surface = _model("MR_intensity_surface_NOT_validated_segmentation", data, (.82, .84, .86), .24)
    surface.SetAndObserveTransformNodeID(transform.GetID())
    return {"volume_path": str(filename), "surface_threshold": threshold,
            "surface_points": data.GetNumberOfPoints(), "surface_cells": data.GetNumberOfCells(),
            "surface_note": "Illustrative MR intensity isosurface with mild Gaussian smoothing and display-only decimation; not validated skin/skull segmentation",
            "surface_gaussian_sigma_voxels": .85, "surface_target_polygon_reduction": .82}


def _pose_from_feedback(pose):
    xyz = np.asarray(pose["position_m"], dtype=float)
    q = np.asarray(pose["orientation_xyzw"], dtype=float)
    if xyz.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(xyz)) or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid recorded ROS feedback pose")
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = xyz * 1000
    return matrix


def _recorded_robot(result):
    # A running panel owns and updates its own robot; never replace its poses.
    if slicer.util.getNodes("UR12e_visual_*"):
        return "Existing UR12e panel meshes, driven by ROS mock-hardware TF feedback"
    snapshot = result.get("robot_feedback_snapshot")
    if not snapshot:
        return "No saved robot TF snapshot supplied; no robot geometry fabricated"
    if snapshot.get("source") != "ros2_mock_hardware" or not snapshot.get("link_transforms"):
        raise ValueError("Robot snapshot must contain saved ros2_mock_hardware link_transforms")
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import ur12e_panel
    manifest = PROJECT / "assets" / "visual_model.json"
    metadata = json.loads(manifest.read_text(encoding="utf-8"))
    count = 0
    for entry in metadata["links"]:
        link = entry["link"]
        if link not in snapshot["link_transforms"]:
            continue
        filename = Path(entry["file"])
        if not filename.is_absolute():
            filename = PROJECT / filename
        if filename.suffix.lower() != ".dae":
            raise ValueError("Unexpected visual asset type in pinned model")
        source = ur12e_panel.collada_polydata(filename)
        origin = np.eye(4)
        origin[:3, :3] = ur12e_panel.rotation_rpy_degrees(*np.rad2deg(entry.get("origin_rpy_rad", [0, 0, 0]))) @ np.diag(np.asarray(entry.get("scale", [1, 1, 1])) * 1000)
        origin[:3, 3] = np.asarray(entry.get("origin_xyz_m", [0, 0, 0])) * 1000
        transform = vtk.vtkTransform()
        transform.SetMatrix(_vtk_matrix(origin))
        converted = vtk.vtkTransformPolyDataFilter()
        converted.SetInputData(source)
        converted.SetTransform(transform)
        converted.Update()
        data = vtk.vtkPolyData()
        data.DeepCopy(converted.GetOutput())
        model = _model("recorded_UR12e_" + link, data, entry.get("color", [.65, .72, .77])[:3])
        parent = _transform("recorded_TF_" + link, _pose_from_feedback(snapshot["link_transforms"][link]))
        model.SetAndObserveTransformNodeID(parent.GetID())
        count += 1
    return "Saved ROS mock-hardware TF snapshot, {} official visual meshes; static replay".format(count)


def _trajectory(result):
    actual, optical = [], []
    for sample in result.get("samples", []):
        if "actual_tip_base_mm" in sample:
            actual.append(_matrix(sample["actual_tip_base_mm"], "sample actual_tip_base_mm")[:3, 3])
        elif "tcp_pose" in sample:
            actual.append(_pose_from_feedback(sample["tcp_pose"])[:3, 3])
        if "optical_tip_base_mm" in sample:
            optical.append(_matrix(sample["optical_tip_base_mm"], "sample optical_tip_base_mm")[:3, 3])
    if len(actual) >= 2:
        _line("recorded_MoveIt_feedback_tip_path", actual, (.15, .9, 1), 1.2)
    if len(optical) >= 2:
        _line("simulated_optical_tip_path", optical, (.85, .35, 1), .75, .7)
    return {"recorded_feedback_path_points": len(actual), "simulated_optical_path_points": len(optical)}


def _camera_and_reference(head):
    if "T_base_reference_mm" in head:
        reference = _matrix(head["T_base_reference_mm"], "T_base_reference_mm")
        _axes("SIMULATED_head_reference", reference, 45, 1.5)
        local = [[-25, 0, 0], [25, 0, 0], [0, 30, 0]]
        _points("SIMULATED_head_reference_markers", _world(reference, local), (.2, 1, .7), "H", 3, 1.0)
        _line("SIMULATED_head_reference_frame", _world(reference, [local[0], local[1], local[2], local[0]]), (.2, 1, .7), 1.8)
    if "T_base_camera_mm" in head:
        camera = _matrix(head["T_base_camera_mm"], "T_base_camera_mm")
        _axes("SIMULATED_optical_camera", camera, 65, 2)
        cube = vtk.vtkCubeSource()
        cube.SetXLength(110)
        cube.SetYLength(40)
        cube.SetZLength(35)
        cube.Update()
        body = _model("SIMULATED_optical_camera_body", cube.GetOutput(), (.18, .22, .3), .8)
        body.SetAndObserveTransformNodeID(_transform("SIMULATED_camera_body_pose", camera).GetID())
        if "T_base_reference_mm" in head:
            _line("SIMULATED_camera_reference_sightline", [camera[:3, 3], reference[:3, 3]], (.3, .7, .65), .45, .35)


def _numeric(value):
    if isinstance(value, (float, int)) and not isinstance(value, bool):
        return "{:.4g}".format(value)
    return str(value)


def _display_report(result, metadata, goal, actual, optical):
    global _dock
    # Find by stable Qt name too: importlib.reload resets the Python _dock
    # reference while an earlier dock can still be alive in the application.
    for previous in slicer.util.findChildren(name="NavigationBenchmarkResultDock"):
        previous.hide()
        slicer.util.mainWindow().removeDockWidget(previous)
        previous.deleteLater()
    _dock = qt.QDockWidget("导航评价 · 仿真结果", slicer.util.mainWindow())
    _dock.setObjectName("NavigationBenchmarkResultDock")
    _dock.setMinimumWidth(390)
    body = qt.QWidget()
    layout = qt.QVBoxLayout(body)
    heading = qt.QLabel("<b>模拟光学 · 独立仿真真值</b><br/>SIMULATION — NO PHYSICAL MEASUREMENT")
    heading.setStyleSheet("font-size: 18px; color: #6a4a00; padding: 6px;")
    heading.setWordWrap(True)
    layout.addWidget(heading)
    note = qt.QLabel("公共 MR 头部示例；F 配准点与 C 检查点均为合成几何点。\n本结果不是实物测量、临床验证或 ASTM 符合性认证。\n青色：MoveIt 仿真反馈；紫色：模拟光学观测；金色：目标。")
    note.setWordWrap(True)
    layout.addWidget(note)
    position_error = float(np.linalg.norm(actual[:3, 3] - goal[:3, 3]))
    optical_error = float(np.linalg.norm(optical[:3, 3] - actual[:3, 3]))
    rotation = actual[:3, :3].T @ goal[:3, :3]
    angle = math.degrees(math.acos(float(np.clip((np.trace(rotation)-1)/2, -1, 1))))
    errors = qt.QLabel("当前记录：\n工具真值 → 目标：{:.3f} mm / {:.3f}°\n模拟光学 → 工具真值：{:.3f} mm".format(position_error, angle, optical_error))
    errors.setStyleSheet("font-size: 16px; padding: 6px;")
    errors.setWordWrap(True)
    layout.addWidget(errors)
    results = qt.QTextBrowser()
    lines = ["坐标约定：影像 RAS mm → 非单位 T_base_image → robot base mm。", "", "记录摘要："]
    for key, value in result.get("summary", {}).items():
        if isinstance(value, (dict, list)):
            value = json.dumps(value, ensure_ascii=False)
        lines.append(str(key) + ": " + _numeric(value))
    lines += ["", "机器人显示：", metadata["robot_display"], "", "注：头部强度等值面仅用于空间示意，未验证为头皮或颅骨分割。"]
    results.setPlainText("\n".join(lines))
    results.setMinimumHeight(220)
    layout.addWidget(results)
    controls = qt.QHBoxLayout()
    for title, mode in [("头部近景", "head"), ("平台全景", "platform")]:
        button = qt.QPushButton(title)
        button.connect("clicked()", lambda checked=False, mode=mode: set_view(result, mode))
        controls.addWidget(button)
    slices_button = qt.QPushButton("MRI 切片")
    slices_button.connect("clicked()", lambda checked=False: show_slices(result))
    controls.addWidget(slices_button)
    layout.addLayout(controls)
    _dock.setWidget(body)
    slicer.util.mainWindow().addDockWidget(qt.Qt.LeftDockWidgetArea, _dock)
    _dock.show()
    view = slicer.app.layoutManager().threeDWidget(0).threeDView()
    annotation = view.cornerAnnotation()
    annotation.SetText(0, "SIMULATED OPTICAL + INDEPENDENT SIMULATION TRUTH\nPublic MR example; synthetic geometric F / C points\nNOT PHYSICAL MEASUREMENT | NOT ASTM COMPLIANCE")
    annotation.SetMaximumFontSize(19)
    annotation.SetMinimumFontSize(14)
    annotation.GetTextProperty().SetColor(1, .85, .55)


def _scene_bounds(result, mode):
    points = []
    for node in slicer.util.getNodesByClass("vtkMRMLModelNode"):
        name = node.GetName() or ""
        include = name == PREFIX + "MR_intensity_surface_NOT_validated_segmentation"
        if mode == "platform":
            include = include or name.startswith(("UR12e_visual_", PREFIX + "recorded_UR12e_", PREFIX + "SIMULATED_optical_camera", PREFIX + "SIMULATED_head_reference"))
        if not include or not node.GetPolyData() or not node.GetDisplayNode() or not node.GetDisplayNode().GetVisibility():
            continue
        bounds = [0.0] * 6
        node.GetRASBounds(bounds)
        if all(np.isfinite(bounds)) and all(bounds[2*i] <= bounds[2*i+1] for i in range(3)):
            points.extend([[x, y, z] for x in bounds[:2] for y in bounds[2:4] for z in bounds[4:6]])
    # Fit the measured end effector, including the complete simplified tool.
    actual = _matrix(result["actual_tip_base_mm"], "actual_tip_base_mm")
    points.extend(_world(actual, [[0, 0, 0], [0, 0, -float(result.get("tool_length_mm", 120))]]))
    if mode == "platform":
        points.append([0, 0, 0])
    values = np.asarray(points)
    return np.min(values, axis=0), np.max(values, axis=0)


def set_view(result, mode="head"):
    layout = slicer.app.layoutManager()
    layout.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutOneUp3DView)
    view = slicer.app.layoutManager().threeDWidget(0).threeDView()
    slicer.app.processEvents()
    lower, upper = _scene_bounds(result, mode)
    center = (lower + upper) / 2
    camera = view.renderWindow().GetRenderers().GetFirstRenderer().GetActiveCamera()
    direction = np.array([.9, -1.4, .28]) if mode == "head" else np.array([1.1, -1.4, .85])
    direction /= np.linalg.norm(direction)
    camera.SetPosition(*(center + direction * max(1800, 3*np.linalg.norm(upper-lower))))
    camera.SetFocalPoint(*center)
    camera.SetViewUp(0, 0, 1)
    camera.SetParallelProjection(True)
    # The two report docks narrow the render viewport: fit projected width as
    # well as height rather than assuming a square/full-window view.
    forward = -direction
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    corners = np.asarray([[x, y, z] for x in [lower[0], upper[0]] for y in [lower[1], upper[1]] for z in [lower[2], upper[2]]]) - center
    width, height = view.renderWindow().GetSize()
    aspect = max(float(width), 1) / max(float(height), 1)
    half_width = np.max(np.abs(corners @ right))
    half_height = np.max(np.abs(corners @ up))
    camera.SetParallelScale(float(max(half_height, half_width/aspect, 50)*1.16))
    view.renderWindow().GetRenderers().GetFirstRenderer().ResetCameraClippingRange()
    view.scheduleRender()
    slicer.app.processEvents()


def show_slices(result=None):
    """Show the actual public MR slices at the selected image-derived goal."""
    if result is None:
        result = slicer.navigationBenchmark["result"]
    slicer.app.layoutManager().setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
    volume = slicer.util.getNode(PREFIX + "public_MR_head")
    slicer.util.setSliceViewerLayers(background=volume)
    slicer.util.resetSliceViews()
    goal = _matrix(result["goal_base_mm"], "goal_base_mm")
    slicer.modules.markups.logic().JumpSlicesToLocation(*[float(v) for v in goal[:3, 3]], True)
    crosshair = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLCrosshairNode")
    if crosshair:
        crosshair.SetCrosshairMode(slicer.vtkMRMLCrosshairNode.ShowBasic)
        crosshair.SetCrosshairRAS(*[float(v) for v in goal[:3, 3]])
    slicer.app.processEvents()
    return {"slice_intersection_base_mm": goal[:3, 3].tolist(), "image_source": volume.GetName(),
            "target_kind": "Synthetic geometric checkpoint; not anatomical annotation"}


def _capture(filename):
    slicer.app.layoutManager().threeDWidget(0).threeDView().renderWindow().Render()
    slicer.app.processEvents()
    if not slicer.util.mainWindow().grab().save(str(filename)):
        raise RuntimeError("Could not save Slicer screenshot: " + str(filename))


def build(result, save=True, result_path=None):
    """Build a simulation result display without clearing or controlling the panel."""
    if result.get("schema_version") != 1 or result.get("mode") != "simulation":
        raise ValueError("Only navigation schema_version=1, mode=simulation is accepted")
    head = result["head"]
    image_to_base = _matrix(head["T_base_image_mm"], "T_base_image_mm")
    if np.allclose(image_to_base, np.eye(4)):
        raise ValueError("Image-to-base must be explicit and non-identity for this navigation scenario")
    goal = _matrix(result["goal_base_mm"], "goal_base_mm")
    actual = _matrix(result["actual_tip_base_mm"], "actual_tip_base_mm")
    optical = _matrix(result["optical_tip_base_mm"], "optical_tip_base_mm")
    _remove_owned_nodes()
    layout = slicer.app.layoutManager()
    if layout.threeDViewCount == 0:
        layout.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutOneUp3DView)
    metadata = {"schema_version": 1, "mode": "simulation", "source_result": str(result_path) if result_path else None,
                "created_at": datetime.datetime.now().astimezone().isoformat(), "independent_truth_kind": "Independent simulation geometry, not a physical measurement",
                "point_kind": "Synthetic geometric points on a public MR example; not anatomical annotations",
                "T_base_image_mm": image_to_base.tolist(), "slicer_version": slicer.app.applicationVersion,
                "sends_robot_commands": False, "clears_existing_scene": False}
    metadata.update(_volume_surface(head, image_to_base))
    fids = _points("synthetic_registration_fiducials_image", head.get("fiducials_image_mm", []), (.2, 1, .35), "F", 3.5)
    targets = _points("synthetic_independent_checkpoints_image", head.get("targets_image_mm", []), (1, .7, .15), "C", 4.5)
    parent = _transform("synthetic_points_image_to_base", image_to_base)
    for node in [fids, targets]:
        if node:
            node.SetAndObserveTransformNodeID(parent.GetID())
    observed = head.get("fiducials_observed_base_mm", [])
    if observed:
        _points("SIMULATED_observed_registration_fiducials", observed, (.8, .4, 1), "ObsF", 2, .8)
        for index, (truth, measured) in enumerate(zip(_world(image_to_base, head.get("fiducials_image_mm", [])), observed)):
            _line("SIMULATED_registration_fiducial_error_" + str(index), [truth, measured], (.8, .4, 1), .6)
    _axes("target_tip", goal, 45, 1.5)
    _axes("robot_feedback_tip", actual, 38, .8)
    _axes("SIMULATED_optical_tip", optical, 30, .7, .8)
    _points("goal_tip_point", [goal[:3, 3]], (1, .7, .15), "Goal", 3.5, 0)
    _points("feedback_tip_point", [actual[:3, 3]], (.15, .9, 1), "Feedback", 2.7, 0)
    _points("SIMULATED_optical_tip_point", [optical[:3, 3]], (.85, .35, 1), "Optical", 2.3, 0)
    _line("target_to_feedback_error", [goal[:3, 3], actual[:3, 3]], (1, .2, .15), 1.3)
    _line("SIMULATED_optical_to_truth_error", [actual[:3, 3], optical[:3, 3]], (.9, .2, 1), .85)
    tool_length = float(result.get("tool_length_mm", 120))
    _line("recorded_feedback_tool", _world(actual, [[0, 0, -tool_length], [0, 0, 0]]), (.15, .9, 1), 2.0)
    _line("SIMULATED_optical_tool", _world(optical, [[0, 0, -tool_length], [0, 0, 0]]), (.85, .35, 1), 1.3, .75)
    _camera_and_reference(head)
    metadata["robot_display"] = _recorded_robot(result)
    metadata.update(_trajectory(result))
    _display_report(result, metadata, goal, actual, optical)
    set_view(result, "head")
    if save:
        folder = Path(result_path).resolve().parent if result_path else DEFAULT_RESULT.parent
        folder.mkdir(parents=True, exist_ok=True)
        scene = folder / "latest_scene.mrb"
        if not slicer.util.saveScene(str(scene)):
            raise RuntimeError("Could not save navigation MRB")
        _capture(folder / "latest_scene.png")
        set_view(result, "platform")
        _capture(folder / "latest_platform.png")
        set_view(result, "head")
        metadata["artifacts"] = {"scene": str(scene), "head_screenshot": str(folder / "latest_scene.png"),
                                 "platform_screenshot": str(folder / "latest_platform.png")}
        (folder / "scene_metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    slicer.navigationBenchmark = {"result": result, "metadata": metadata}
    return metadata


def run(result_json_path=None, save=True):
    path = Path(result_json_path) if result_json_path else DEFAULT_RESULT
    result = json.loads(path.read_text(encoding="utf-8-sig"))
    return build(result, save=save, result_path=path)


def _startup():
    try:
        path = os.environ.get("UR12E_NAVBENCH_RESULT") or str(DEFAULT_RESULT)
        metadata = run(path)
        destination = PROJECT / "runtime" / "navigation_benchmark_viewer.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps({"pid": os.getpid(), "started_at": metadata["created_at"], "result": str(path),
                                           "simulation_only": True, "starts_bridge": False}, ensure_ascii=False, indent=2), encoding="utf-8")
        slicer.util.mainWindow().setWindowTitle("Navigation benchmark | SIMULATION RESULT VIEWER")
    except Exception:
        destination = PROJECT / "runtime" / "navigation_benchmark_viewer_startup_error.txt"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(traceback.format_exc(), encoding="utf-8")
        print(traceback.format_exc())


if __name__ == "__main__":
    qt.QTimer.singleShot(1000, _startup)
