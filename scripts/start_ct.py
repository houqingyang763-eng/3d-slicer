from pathlib import Path
import sys
import qt
import slicer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def initialize():
    import slicer_bridge
    slicer_bridge.start()
    scene = ROOT / "output" / "scenes" / "CT_threshold_practice.mrb"
    if not slicer.util.loadScene(str(scene)):
        raise RuntimeError("Could not load CT practice scene")
    slicer.util.selectModule("SegmentEditor")
    editor = slicer.modules.segmenteditor.widgetRepresentation().self().editor
    editor.setSegmentationNode(slicer.util.getNode("ThresholdPractice_NotValidatedAnatomy"))
    editor.setSourceVolumeNode(slicer.util.getNode("CTBrain"))
    slicer.util.mainWindow().resize(1480, 920)


qt.QTimer.singleShot(1000, initialize)
