"""UR12e URSim only, on this task's isolated local Docker endpoint.

The startup wrapper must verify that URSIM_IP belongs to the recorded local
URSim container before invoking this launch. This fixed endpoint is deliberately
not a launch argument; this file does not provide physical-robot access.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


URSIM_IP = "192.168.62.2"
REVERSE_IP = "192.168.62.1"
CONTROLLER = "scaled_joint_trajectory_controller"


def generate_launch_description():
    package = FindPackageShare("ur12e_slicer_bridge")
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": "ur12e", "robot_ip": URSIM_IP, "use_mock_hardware": "false",
            "reverse_ip": REVERSE_IP,
            "initial_joint_controller": CONTROLLER, "launch_rviz": "false",
            "headless_mode": "false", "launch_dashboard_client": "true",
            "use_tool_communication": "false",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])),
        launch_arguments={
            "ur_type": "ur12e", "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false", "use_sim_time": "false",
        }.items(),
    )
    bridge = Node(
        package="ur12e_slicer_bridge", executable="file_bridge", name="ur12e_slicer_bridge",
        output="screen", parameters=[
            PathJoinSubstitution([package, "config", "bridge.yaml"]),
            {"runtime_dir": LaunchConfiguration("runtime_dir"),
             "feedback_source": "ros2_ursim", "controller_name": CONTROLLER},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("runtime_dir", description="Absolute task-owned URSim file-bridge directory"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        control, moveit, bridge,
    ])
