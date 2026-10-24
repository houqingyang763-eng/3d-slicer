"""Launch the dedicated UR12e simulation panel inside 3D Slicer."""
from pathlib import Path
import sys
import traceback

import qt
import slicer

PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))


def initialize():
    try:
        import task_bridge
        import ur12e_panel
        task_bridge.start()
        ur12e_panel.start()
        obsolete_error = PROJECT / "runtime" / "slicer_startup_error.txt"
        if obsolete_error.exists():
            obsolete_error.unlink()
    except Exception:
        destination = PROJECT / "runtime" / "slicer_startup_error.txt"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(traceback.format_exc(), encoding="utf-8")
        print(traceback.format_exc())


qt.QTimer.singleShot(1000, initialize)
