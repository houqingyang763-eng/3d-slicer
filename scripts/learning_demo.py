"""Two reproducible teaching scenes. The target and tracked tool are simulated."""
from pathlib import Path
from datetime import datetime
import json
import math

import numpy as np
import qt
import slicer
import vtk

ROOT = Path(__file__).resolve().parents[1]
SCENES = ROOT / "output" / "scenes"


def save_scene(name):
    SCENES.mkdir(parents=True, exist_ok=True)
    path = SCENES / name
    if not slicer.util.saveScene(str(path)):
        raise RuntimeError(f"Could not save {path}")
    return str(path)


def build_ct():
    if slicer.util.getNodesByClass("vtkMRMLVolumeNode"):
        raise RuntimeError("Build the baseline scenes in an empty classroom instance.")
    volume = slicer.util.loadVolume(str(ROOT / "data" / "CT-brain.nrrd"), {"name": "CTBrain"})
    if not volume:
        raise RuntimeError("CT loading failed")
    display = volume.GetDisplayNode()
    display.AutoWindowLevelOff()
    display.SetWindowLevel(1800, 500)
    segmentation = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLSegmentationNode", "ThresholdPractice_NotValidatedAnatomy")
    segmentation.CreateDefaultDisplayNodes()
    segmentation.SetReferenceImageGeometryParameterFromVolumeNode(volume)
    segment = segmentation.GetSegmentation().AddEmptySegment("", "HU_above_250_practice", [0.95, 0.88, 0.66])
    values = slicer.util.arrayFromVolume(volume)
    mask = (values >= 250).astype(np.uint8)
    slicer.util.updateSegmentBinaryLabelmapFromArray(mask, segmentation, segment, volume)
    segmentation.CreateClosedSurfaceRepresentation()
    segmentation.GetDisplayNode().SetOpacity3D(0.65)
    # Export one surface model to demonstrate the distinction between segmentation and model.
    surface = vtk.vtkPolyData()
    segmentation.GetClosedSurfaceRepresentation(segment, surface)
    model = slicer.modules.models.logic().AddModel(surface)
    model.SetName("CT_threshold_surface_model")
    model.GetDisplayNode().SetColor(0.95, 0.88, 0.66)
    model.GetDisplayNode().SetVisibility(False)
    slicer.util.setSliceViewerLayers(background=volume)
    slicer.app.layoutManager().setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
    slicer.util.resetThreeDViews()
    slicer.util.selectModule("SegmentEditor")
    path = save_scene("CT_threshold_practice.mrb")
    return {"scene": path, "threshold": 250, "voxels": int(mask.sum()), "mesh_points": surface.GetNumberOfPoints()}


def build_mri():
    slicer.mrmlScene.Clear(0)  # Only called after saving our newly built CT baseline.
    volume = slicer.util.loadVolume(str(ROOT / "data" / "MR-head.nrrd"), {"name": "MRHead"})
    if not volume:
        raise RuntimeError("MRI loading failed")
    logic = slicer.modules.volumerendering.logic()
    rendering = logic.CreateDefaultVolumeRenderingNodes(volume)
    preset = logic.GetPresetByName("MR-Default")
    if preset:
        rendering.GetVolumePropertyNode().Copy(preset)
    rendering.SetVisibility(True)
    # Make the head translucent enough to see our illustrative internal target.
    opacity = rendering.GetVolumePropertyNode().GetVolumeProperty().GetScalarOpacity()
    for i in range(opacity.GetSize()):
        point = [0.0] * 4
        opacity.GetNodeValue(i, point)
        point[1] *= 0.04
        opacity.SetNodeValue(i, point)
    if rendering.GetROINode():
        rendering.GetROINode().GetDisplayNode().SetVisibility(False)
    bounds = [0.0] * 6
    volume.GetRASBounds(bounds)
    center = np.array([(bounds[i] + bounds[i + 1]) / 2 for i in (0, 2, 4)])
    target_position = center + np.array([15.0, -5.0, 10.0])
    target = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "PracticeTarget")
    target.AddControlPoint(vtk.vtkVector3d(*target_position))
    target.SetNthControlPointLabel(0, "Demo target")
    target.SetLocked(True)
    target.GetDisplayNode().SetSelectedColor(1.0, 0.78, 0.05)
    target.GetDisplayNode().SetGlyphScale(3.0)

    sphere = vtk.vtkSphereSource()
    sphere.SetCenter(*target_position)
    sphere.SetRadius(4.0)
    sphere.SetThetaResolution(32)
    sphere.SetPhiResolution(32)
    target_model = slicer.modules.models.logic().AddModel(sphere.GetOutputPort())
    target_model.SetName("IllustrativeTargetSphere")
    target_model.GetDisplayNode().SetColor(1.0, 0.65, 0.05)

    transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "SimulatedToolToImage")
    initial_tip = target_position + np.array([45.0, -30.0, 45.0])
    z = (target_position - initial_tip) / np.linalg.norm(target_position - initial_tip)
    x = np.cross(np.array([0.0, 1.0, 0.0]), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack((x, y, z))
    matrix[:3, 3] = initial_tip
    slicer.util.updateTransformMatrixFromArray(transform, matrix)
    transform.SetAttribute("PracticeInitialMatrix", json.dumps(matrix.tolist()))

    line = vtk.vtkLineSource()
    line.SetPoint1(0, 0, -75)
    line.SetPoint2(0, 0, 0)
    tube = vtk.vtkTubeFilter()
    tube.SetInputConnection(line.GetOutputPort())
    tube.SetRadius(1.8)
    tube.SetNumberOfSides(24)
    tube.CappingOn()
    tool = slicer.modules.models.logic().AddModel(tube.GetOutputPort())
    tool.SetName("SimulatedToolModel")
    tool.GetDisplayNode().SetColor(0.2, 0.85, 1.0)
    tool.SetAndObserveTransformNodeID(transform.GetID())
    tip = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "SimulatedToolTip")
    tip.AddControlPoint(vtk.vtkVector3d(0, 0, 0))
    tip.SetNthControlPointLabel(0, "Simulated tip")
    tip.SetAndObserveTransformNodeID(transform.GetID())
    tip.SetLocked(True)
    tip.GetDisplayNode().SetSelectedColor(0.1, 0.9, 1.0)

    distance = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsLineNode", "TipToTargetDistance")
    distance.AddControlPoint(vtk.vtkVector3d(*initial_tip))
    distance.AddControlPoint(vtk.vtkVector3d(*target_position))
    distance.SetLocked(True)
    distance.GetDisplayNode().SetSelectedColor(0.3, 1.0, 0.4)
    distance.GetDisplayNode().SetTextScale(2.0)
    distance.GetDisplayNode().SetPropertiesLabelVisibility(False)
    distance.GetDisplayNode().SetPointLabelsVisibility(False)
    distance.GetDisplayNode().SetGlyphScale(0.6)
    slicer.util.setSliceViewerLayers(background=volume)
    slicer.app.layoutManager().setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
    slicer.modules.markups.logic().JumpSlicesToLocation(*target_position, True)
    slicer.util.resetThreeDViews()
    slicer.util.selectModule("Data")
    view = slicer.app.layoutManager().threeDWidget(0).mrmlViewNode()
    view.SetBackgroundColor(0.12, 0.14, 0.17)
    view.SetBackgroundColor2(0.12, 0.14, 0.17)
    view.SetBoxVisible(False)
    attach_controls()
    path = save_scene("MRI_navigation_practice.mrb")
    return {"scene": path, "target_RAS_mm": target_position.tolist(), "initial_tip_RAS_mm": initial_tip.tolist(), "initial_distance_mm": float(np.linalg.norm(initial_tip - target_position))}


class PracticeControls:
    def __init__(self):
        self.transform = slicer.util.getNode("SimulatedToolToImage")
        self.target = slicer.util.getNode("PracticeTarget")
        self.distance = slicer.util.getNode("TipToTargetDistance")
        self.dock = qt.QDockWidget("课堂练习", slicer.util.mainWindow())
        body = qt.QWidget()
        layout = qt.QVBoxLayout(body)
        intro = qt.QLabel("公开示例 MRI\n虚构目标 · 模拟工具")
        intro.setStyleSheet("font-size: 15px; padding: 8px;")
        layout.addWidget(intro)
        self.reading = qt.QLabel()
        self.reading.setStyleSheet("font-size: 17px; padding: 10px; color: #23665b;")
        layout.addWidget(self.reading)
        for label, callback in [
            ("向目标移动 10 mm", lambda: self.move(-10)),
            ("远离目标 10 mm", lambda: self.move(10)),
            ("横向移动 10 mm", self.move_sideways),
            ("重置工具位置", self.reset),
            ("切片定位到目标", self.jump_to_target),
            ("显示 / 隐藏三维头部", self.toggle_head),
            ("保存当前练习", self.save_copy),
        ]:
            button = qt.QPushButton(label)
            button.setMinimumHeight(36)
            button.connect("clicked()", callback)
            layout.addWidget(button)
        self.note = qt.QLabel("左侧 Data：查看对象\nTransforms：查看工具变换\n此场景没有接入定位硬件")
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        layout.addStretch(1)
        self.dock.setWidget(body)
        self.dock.setMinimumWidth(230)
        slicer.util.mainWindow().addDockWidget(qt.Qt.RightDockWidgetArea, self.dock)
        self.observer = self.transform.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, self.update)
        self.update()

    def target_position(self):
        position = [0.0] * 3
        self.target.GetNthControlPointPositionWorld(0, position)
        return np.asarray(position)

    def matrix(self):
        return slicer.util.arrayFromTransformMatrix(self.transform, toWorld=True)

    def update(self, *unused):
        matrix = self.matrix()
        tip = matrix[:3, 3]
        target = self.target_position()
        vector = target - tip
        distance = float(np.linalg.norm(vector))
        if distance > 1e-6:
            axis = matrix[:3, 2] / np.linalg.norm(matrix[:3, 2])
            angle = math.degrees(math.acos(float(np.clip(np.dot(axis, vector / distance), -1, 1))))
            direction_text = f"指向夹角 {angle:.1f}°"
        else:
            direction_text = "工具尖端与示例目标重合"
        self.distance.SetNthControlPointPositionWorld(0, *tip)
        self.distance.SetNthControlPointPositionWorld(1, *target)
        self.reading.setText(f"距目标 {distance:.1f} mm\n{direction_text}")

    def move(self, delta):
        matrix = self.matrix()
        away = matrix[:3, 3] - self.target_position()
        current = np.linalg.norm(away)
        if current < 1e-6:
            away = -matrix[:3, 2]
        else:
            away /= current
        matrix[:3, 3] = self.target_position() + away * max(0.0, current + delta)
        slicer.util.updateTransformMatrixFromArray(self.transform, matrix)

    def move_sideways(self):
        matrix = self.matrix()
        matrix[:3, 3] += matrix[:3, 0] * 10
        slicer.util.updateTransformMatrixFromArray(self.transform, matrix)

    def reset(self):
        matrix = np.asarray(json.loads(self.transform.GetAttribute("PracticeInitialMatrix")))
        slicer.util.updateTransformMatrixFromArray(self.transform, matrix)

    def jump_to_target(self):
        slicer.modules.markups.logic().JumpSlicesToLocation(*self.target_position(), True)

    def save_copy(self):
        path = save_scene(f"My_practice_{datetime.now():%Y%m%d_%H%M%S}.mrb")
        self.note.setText(f"已保存：\n{Path(path).name}")

    def toggle_head(self):
        for display in slicer.util.getNodesByClass("vtkMRMLVolumeRenderingDisplayNode"):
            display.SetVisibility(not display.GetVisibility())


def attach_controls():
    old = getattr(slicer, "classroomPractice", None)
    if old:
        old.transform.RemoveObserver(old.observer)
        slicer.util.mainWindow().removeDockWidget(old.dock)
        old.dock.deleteLater()
    slicer.classroomPractice = PracticeControls()
    return slicer.classroomPractice
