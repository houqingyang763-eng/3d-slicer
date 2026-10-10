"""Entry point for the classroom Slicer instance."""
from pathlib import Path
import json
import sys
import traceback

import qt
import slicer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def initialize():
    try:
        import slicer_bridge
        slicer_bridge.start()
        scene = ROOT / "output" / "scenes" / "MRI_navigation_practice.mrb"
        if scene.exists() and not slicer.mrmlScene.GetFirstNodeByName("PracticeTarget"):
            if not slicer.util.loadScene(str(scene)):
                raise RuntimeError("Could not load practice scene")
            import learning_demo
            learning_demo.attach_controls()
        slicer.util.mainWindow().resize(1480, 920)
    except Exception:
        error = ROOT / "work" / "startup_error.txt"
        error.write_text(traceback.format_exc(), encoding="utf-8")
        print(traceback.format_exc())


qt.QTimer.singleShot(1000, initialize)
