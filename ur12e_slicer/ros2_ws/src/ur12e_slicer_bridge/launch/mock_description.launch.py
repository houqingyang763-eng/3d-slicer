"""Official UR description with this mock demo's explicit, bent-elbow home."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import xacro


def robot_description(_context):
    driver = Path(get_package_share_directory("ur_robot_driver"))
    bridge = Path(get_package_share_directory("ur12e_slicer_bridge"))
    # The official ur_rsp launch does not forward initial_positions_file to
    # xacro. Process that same official model here with the explicit mapping.
    # Both robot type and mock mode are deliberately fixed for this project.
    model = xacro.process_file(str(driver / "urdf" / "ur.urdf.xacro"), mappings={
        "name": "ur12e", "ur_type": "ur12e", "use_mock_hardware": "true",
        "mock_sensor_commands": "false", "safety_limits": "true",
        "initial_positions_file": str(bridge / "config" / "mock_initial_positions.yaml"),
    })
    return [Node(package="robot_state_publisher", executable="robot_state_publisher",
                 output="screen", parameters=[{"robot_description": model.toxml()}])]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=robot_description)])
