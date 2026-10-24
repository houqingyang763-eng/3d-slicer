"""Jazzy UR official driver + MoveIt, deliberately limited to mock hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    runtime = LaunchConfiguration("runtime_dir")
    package = FindPackageShare("ur12e_slicer_bridge")
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])),
        launch_arguments={
            "ur_type": "ur12e", "robot_ip": "127.0.0.1", "use_mock_hardware": "true",
            "initial_joint_controller": "joint_trajectory_controller", "launch_rviz": "false",
            "launch_dashboard_client": "false", "use_tool_communication": "false",
            "description_launchfile": PathJoinSubstitution([package, "launch", "mock_description.launch.py"]),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])),
        launch_arguments={"ur_type": "ur12e", "launch_rviz": LaunchConfiguration("launch_rviz"),
                          "launch_servo": "false", "use_sim_time": "false"}.items(),
    )
    bridge = Node(
        package="ur12e_slicer_bridge", executable="file_bridge", name="ur12e_slicer_bridge", output="screen",
        parameters=[PathJoinSubstitution([package, "config", "bridge.yaml"]), {"runtime_dir": runtime}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("runtime_dir", description="Absolute WSL path to the shared Windows ur12e_slicer/runtime folder"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        control, moveit, bridge,
    ])
