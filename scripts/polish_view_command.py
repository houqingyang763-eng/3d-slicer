import importlib
import learning_demo
importlib.reload(learning_demo)
for display in slicer.util.getNodesByClass("vtkMRMLVolumeRenderingDisplayNode"):
    opacity = display.GetVolumePropertyNode().GetVolumeProperty().GetScalarOpacity()
    for i in range(opacity.GetSize()):
        point = [0.0] * 4
        opacity.GetNodeValue(i, point)
        point[1] *= 0.008 / 0.22
        opacity.SetNodeValue(i, point)
    if display.GetROINode():
        display.GetROINode().GetDisplayNode().SetVisibility(False)
distance = slicer.util.getNode("TipToTargetDistance")
distance.GetDisplayNode().SetPropertiesLabelVisibility(False)
distance.GetDisplayNode().SetPointLabelsVisibility(False)
distance.GetDisplayNode().SetGlyphScale(0.6)
view = slicer.app.layoutManager().threeDWidget(0).mrmlViewNode()
view.SetBackgroundColor(0.12, 0.14, 0.17)
view.SetBackgroundColor2(0.12, 0.14, 0.17)
learning_demo.attach_controls()
learning_demo.save_scene("MRI_navigation_practice.mrb")
slicer.app.processEvents()
slicer.util.mainWindow().grab().save(str(ROOT / "output" / "screenshots" / "MRI_practice.png"))
_result = {"polished": True}
