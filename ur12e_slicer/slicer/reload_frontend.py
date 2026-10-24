"""Development-only command sent through the existing local Slicer bridge."""
import importlib
import ur12e_panel
if hasattr(slicer, "ur12eSimulation"):
    slicer.ur12eSimulation.stop()
    del slicer.ur12eSimulation
importlib.reload(ur12e_panel)
panel = ur12e_panel.start()
panel.load_visual_meshes()
_result = {"loaded": True, "mesh_links": list(panel.mesh_transforms), "scene_nodes": slicer.mrmlScene.GetNumberOfNodes(), "screenshot": panel.capture(ur12e_panel.OUTPUT / "frontend_waiting_for_ros.png")}
