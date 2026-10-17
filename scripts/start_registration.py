"""Launch the registration classroom and local command bridge."""
from pathlib import Path
import sys
import qt
import slicer
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
def initialize():
    import slicer_bridge
    import registration_practice
    slicer_bridge.start()
    registration_practice.start()
qt.QTimer.singleShot(1000,initialize)
