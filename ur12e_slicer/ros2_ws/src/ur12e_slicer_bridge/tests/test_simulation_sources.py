"""Offline checks for simulation source identity and fixed URSim routing.

ROS/Qt objects below are inert import stubs. These checks cannot establish a
running container, controller connection, trajectory execution or GUI behavior.
"""
import importlib.util
import os
from pathlib import Path
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch


PACKAGE = Path(__file__).resolve().parents[1]
PROJECT = PACKAGE.parents[2]


def load_module(path, name, stubs):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", stubs):
        spec.loader.exec_module(module)
    return module


def stub_module(name, **members):
    module = ModuleType(name)
    module.__dict__.update(members)
    return module


class NodeStub:
    overrides = {}

    def __init__(self, _name):
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_subscription(self, *_args):
        return MagicMock()

    def create_client(self, *_args):
        return MagicMock()

    def create_timer(self, *_args):
        return MagicMock()

    def get_logger(self):
        return MagicMock()


def node_stubs():
    modules = {
        "rclpy": {},
        "action_msgs": {}, "action_msgs.msg": {"GoalStatus": MagicMock()},
        "geometry_msgs": {}, "geometry_msgs.msg": {"Pose": MagicMock()},
        "moveit_msgs": {}, "moveit_msgs.action": {"ExecuteTrajectory": MagicMock()},
        "moveit_msgs.msg": {name: MagicMock() for name in (
            "AttachedCollisionObject", "Constraints", "MoveItErrorCodes", "OrientationConstraint",
            "PlanningScene", "PositionConstraint", "RobotState")},
        "moveit_msgs.srv": {name: MagicMock() for name in ("ApplyPlanningScene", "GetMotionPlan", "GetPositionFK")},
        "rclpy.action": {"ActionClient": MagicMock()},
        "rclpy.executors": {"ExternalShutdownException": RuntimeError},
        "rclpy.node": {"Node": NodeStub},
        "rclpy.qos": {"qos_profile_sensor_data": object()},
        "rclpy.time": {"Time": MagicMock()},
        "sensor_msgs": {}, "sensor_msgs.msg": {"JointState": MagicMock()},
        "shape_msgs": {}, "shape_msgs.msg": {"SolidPrimitive": MagicMock()},
        "tf2_ros": {"Buffer": MagicMock(), "TransformException": RuntimeError, "TransformListener": MagicMock()},
    }
    return {name: stub_module(name, **members) for name, members in modules.items()}


class SourceIdentityTests(unittest.TestCase):
    @staticmethod
    def load_panel():
        return load_module(PROJECT / "slicer/ur12e_panel.py", "_source_test_panel", {
            "numpy": stub_module("numpy", eye=lambda _size: None),
            "qt": stub_module("qt"), "slicer": stub_module("slicer"), "vtk": stub_module("vtk"),
        })

    @classmethod
    def setUpClass(cls):
        cls.node = load_module(PACKAGE / "ur12e_slicer_bridge/node.py",
                               "ur12e_slicer_bridge._source_test_node", node_stubs())
        cls.panel = cls.load_panel()

    def tearDown(self):
        NodeStub.overrides = {}

    def test_unconfigured_bridge_keeps_mock_source_and_controller(self):
        with tempfile.TemporaryDirectory() as folder:
            NodeStub.overrides = {"runtime_dir": folder}
            bridge = self.node.FileBridge()
            self.assertEqual(bridge.state["source"], "ros2_mock_hardware")
            self.assertEqual(bridge.p("controller_name"), "joint_trajectory_controller")

    def test_ursim_bridge_reports_distinct_source(self):
        with tempfile.TemporaryDirectory() as folder:
            NodeStub.overrides = {"runtime_dir": folder, "feedback_source": "ros2_ursim",
                                  "controller_name": "scaled_joint_trajectory_controller"}
            bridge = self.node.FileBridge()
            self.assertEqual(bridge.state["source"], "ros2_ursim")
            self.assertIn("URSim", bridge.backend_description)
            self.assertEqual(bridge.p("controller_name"), "scaled_joint_trajectory_controller")

    def test_physical_or_unknown_sources_fail_before_runtime_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            destination = Path(folder) / "must-not-be-created"
            for source in ("ros2_real_hardware", "test_fixture", "", None, True, {}):
                with self.subTest(source=source):
                    NodeStub.overrides = {"runtime_dir": str(destination), "feedback_source": source}
                    with self.assertRaises(ValueError):
                        self.node.FileBridge()
                    self.assertFalse(destination.exists())

    def test_panel_names_each_simulator_and_rejects_other_sources(self):
        self.assertIn("mock", self.panel.feedback_source_label("ros2_mock_hardware"))
        self.assertIn("URSim", self.panel.feedback_source_label("ros2_ursim"))
        for source in ("ros2_real_hardware", "test_fixture", None, True, {}):
            with self.subTest(source=source), self.assertRaises(ValueError):
                self.panel.feedback_source_label(source)

    def test_default_mock_panel_keeps_original_runtime(self):
        with patch.dict(os.environ, {}, clear=True):
            panel_module = self.load_panel()
        self.assertEqual(panel_module.CONTROL_RUNTIME, panel_module.RUNTIME.resolve())

    def test_ursim_panel_override_only_moves_control_files(self):
        with tempfile.TemporaryDirectory() as folder:
            control = Path(folder) / "Linux bridge with spaces"
            with patch.dict(os.environ, {"UR12E_CONTROL_RUNTIME_DIR": str(control)}):
                panel_module = self.load_panel()
            self.assertEqual(panel_module.CONTROL_RUNTIME, control.resolve())
            self.assertEqual(panel_module.RUNTIME, PROJECT / "runtime")
            panel = panel_module.UR12ePanel.__new__(panel_module.UR12ePanel)
            panel.feedback_error = None
            panel.state = {"ready": True, "timestamp": panel_module.time.time(), "status": "idle"}
            panel.current_target = lambda: {"position_m": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1]}
            panel.execute_button = MagicMock()
            panel.status_label = MagicMock()
            panel.submit_buttons = []
            with patch.object(panel_module, "atomic_json") as write:
                self.assertIsNotNone(panel.send("plan"))
            self.assertEqual(write.call_args[0][0], control.resolve() / "target.json")
            control.mkdir(parents=True)
            (control / "validation.lock").write_text("owned", encoding="utf-8")
            with patch.object(panel_module, "atomic_json") as write:
                self.assertIsNone(panel.send("plan"))
            write.assert_not_called()


class Capture:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Description(Capture):
    @property
    def entities(self):
        return self.args[0]


class Declaration(Capture):
    pass


class Include(Capture):
    pass


class RosNode(Capture):
    pass


class FixedUrSimLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = load_module(PACKAGE / "launch/ursim.launch.py", "_source_test_ursim_launch", {
            "launch": stub_module("launch", LaunchDescription=Description),
            "launch.actions": stub_module("launch.actions", DeclareLaunchArgument=Declaration,
                                           IncludeLaunchDescription=Include),
            "launch.launch_description_sources": stub_module("launch.launch_description_sources",
                                                               PythonLaunchDescriptionSource=Capture),
            "launch.substitutions": stub_module("launch.substitutions", LaunchConfiguration=Capture,
                                                 PathJoinSubstitution=Capture),
            "launch_ros": stub_module("launch_ros"),
            "launch_ros.actions": stub_module("launch_ros.actions", Node=RosNode),
            "launch_ros.substitutions": stub_module("launch_ros.substitutions", FindPackageShare=Capture),
        })

    def test_endpoint_and_model_cannot_be_replaced_by_launch_arguments(self):
        entities = self.launch.generate_launch_description().entities
        declarations = {item.args[0] for item in entities if isinstance(item, Declaration)}
        self.assertEqual(declarations, {"runtime_dir", "launch_rviz"})
        control = next(item for item in entities if isinstance(item, Include))
        arguments = dict(control.kwargs["launch_arguments"])
        self.assertEqual(arguments["robot_ip"], "192.168.62.2")
        self.assertEqual(arguments["reverse_ip"], "192.168.62.1")
        self.assertEqual(arguments["ur_type"], "ur12e")
        self.assertEqual(arguments["use_mock_hardware"], "false")
        self.assertNotIn("description_launchfile", arguments)

    def test_scaled_controller_matches_driver_and_bridge_execution(self):
        entities = self.launch.generate_launch_description().entities
        control = next(item for item in entities if isinstance(item, Include))
        arguments = dict(control.kwargs["launch_arguments"])
        bridge = next(item for item in entities if isinstance(item, RosNode))
        overrides = bridge.kwargs["parameters"][-1]
        self.assertEqual(arguments["initial_joint_controller"], "scaled_joint_trajectory_controller")
        self.assertEqual(overrides["controller_name"], arguments["initial_joint_controller"])
        self.assertEqual(overrides["feedback_source"], "ros2_ursim")


if __name__ == "__main__":
    unittest.main()
