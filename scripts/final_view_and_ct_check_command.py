import importlib
import learning_demo
importlib.reload(learning_demo)
for display in slicer.util.getNodesByClass("vtkMRMLVolumeRenderingDisplayNode"):
    opacity = display.GetVolumePropertyNode().GetVolumeProperty().GetScalarOpacity()
    for i in range(opacity.GetSize()):
        point = [0.0] * 4
        opacity.GetNodeValue(i, point)
        point[1] *= 0.04 / 0.008
        opacity.SetNodeValue(i, point)
view = slicer.app.layoutManager().threeDWidget(0).mrmlViewNode()
view.SetBoxVisible(False)
learning_demo.save_scene("MRI_navigation_practice.mrb")
controls = slicer.classroomPractice
controls.transform.RemoveObserver(controls.observer)
slicer.util.mainWindow().removeDockWidget(controls.dock)
controls.dock.deleteLater()
del slicer.classroomPractice
slicer.mrmlScene.Clear(0)
assert slicer.util.loadScene(str(ROOT / "output" / "scenes" / "CT_threshold_practice.mrb"))
slicer.util.selectModule("SegmentEditor")
editor = slicer.modules.segmenteditor.widgetRepresentation().self().editor
editor.setSegmentationNode(slicer.util.getNode("ThresholdPractice_NotValidatedAnatomy"))
editor.setSourceVolumeNode(slicer.util.getNode("CTBrain"))
slicer.util.resetThreeDViews()
slicer.app.processEvents()
assert slicer.util.mainWindow().grab().save(str(ROOT / "output" / "screenshots" / "CT_practice.png"))
slicer.mrmlScene.Clear(0)
assert slicer.util.loadScene(str(ROOT / "output" / "scenes" / "MRI_navigation_practice.mrb"))
slicer.util.selectModule("Data")
learning_demo.attach_controls()
slicer.app.processEvents()
assert slicer.util.mainWindow().grab().save(str(ROOT / "output" / "screenshots" / "MRI_practice.png"))
_result = {"CT_loaded_in_GUI": True, "MRI_restored": True}
