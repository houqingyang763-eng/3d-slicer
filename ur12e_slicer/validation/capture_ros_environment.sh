#!/usr/bin/env bash
# Read-only evidence from the actual running graph on this task's domain.
set -Eeuo pipefail
OUTPUT="${1:?An output directory is required}"
mkdir -p -- "$OUTPUT"
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1
set +u
source /opt/ros/jazzy/setup.bash
if [[ -n "${UR12E_BUILD_DIR:-}" ]]; then
  source "$UR12E_BUILD_DIR/install/setup.bash"
elif ! ros2 pkg prefix ur12e_slicer_bridge >/dev/null 2>&1; then
  printf 'Source the project workspace first or set UR12E_BUILD_DIR.\n' >&2
  exit 2
fi
set -u
date --iso-8601=seconds > "$OUTPUT/captured_at.txt"
python3 - "$OUTPUT" <<'PY'
import json
from pathlib import Path
import sys
import time
import rclpy
from rclpy.action.graph import get_action_names_and_types

output = Path(sys.argv[1])
rclpy.init()
node = rclpy.create_node("ur12e_validation_graph_observer")
deadline = time.monotonic() + 3
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
nodes = sorted(node.get_node_names())
services = sorted(node.get_service_names_and_types())
actions = sorted(get_action_names_and_types(node))
(output / "nodes.txt").write_text("\n".join(nodes) + "\n")
for name, entries in (("services", services), ("actions", actions)):
    (output / f"{name}.txt").write_text("\n".join(f"{key} [{', '.join(types)}]" for key, types in entries) + "\n")
(output / "graph.json").write_text(json.dumps({"nodes": nodes, "services": services, "actions": actions}, indent=2))
node.destroy_node()
rclpy.shutdown()
PY
ros2 control list_hardware_components -v > "$OUTPUT/hardware_components.txt"
ros2 control list_controllers -v > "$OUTPUT/controllers.txt"
ros2 param get /robot_state_publisher robot_description > "$OUTPUT/robot_description.txt"
dpkg-query -W ros-jazzy-ur-robot-driver ros-jazzy-ur-description \
  ros-jazzy-ur-moveit-config ros-jazzy-moveit-ros-move-group ros-jazzy-controller-manager \
  > "$OUTPUT/package_versions.tsv"
ps -eo pid,ppid,pgid,args > "$OUTPUT/processes.txt"
python3 - "$OUTPUT" <<'PY'
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

output = Path(sys.argv[1])
description = (output / "robot_description.txt").read_text()
start = description.find("<?xml")
if start < 0:
    start = description.find("<robot")
assert start >= 0, "No actual robot_description returned"
model = ET.fromstring(description[start:])
plugins = [node.text for node in model.findall("./ros2_control/hardware/plugin")]
initial = {node.attrib["name"]: float(node.find("./state_interface[@name='position']/param[@name='initial_value']").text)
           for node in model.findall("./ros2_control/joint")}
checks = {
    "official_mock_plugin_in_actual_urdf": plugins == ["mock_components/GenericSystem"],
    "real_hardware_plugin_absent": "ur_robot_driver/URPositionHardwareInterface" not in plugins,
    "mock_hardware_active": "active" in (output / "hardware_components.txt").read_text(),
    "jtc_active": any("joint_trajectory_controller" in line and "active" in line and "inactive" not in line
                      for line in (output / "controllers.txt").read_text().splitlines()),
    "planning_service_present": "/plan_kinematic_path [moveit_msgs/srv/GetMotionPlan]" in (output / "services.txt").read_text(),
    "execution_action_present": "/execute_trajectory [moveit_msgs/action/ExecuteTrajectory]" in (output / "actions.txt").read_text(),
}
summary = {"domain_id": 62, "localhost_only": True, "plugins": plugins, "initial_positions_rad": initial,
           "checks": checks, "passed": all(checks.values())}
(output / "environment_summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
assert summary["passed"], "Actual ROS graph evidence checks failed"
PY
