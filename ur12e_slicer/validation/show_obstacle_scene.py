"""Run through task_client.py after scene_tool.py confirms the scene."""
import importlib
import sys
sys.path.insert(0, str(PROJECT / "validation"))
import obstacle_display
importlib.reload(obstacle_display)
_result = obstacle_display.sync()
