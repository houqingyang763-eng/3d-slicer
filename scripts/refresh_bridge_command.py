import importlib
import slicer_bridge
old_bridge = slicer.classroomBridge
old_bridge.timer.stop()
del slicer.classroomBridge
importlib.reload(slicer_bridge)
slicer_bridge.start()
_result = {"active_session": slicer.classroomBridge.session}
