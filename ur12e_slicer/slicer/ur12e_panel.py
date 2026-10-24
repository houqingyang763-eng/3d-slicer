"""UR12e simulation console. Run with Slicer's embedded Python.

The displayed robot always comes from feedback in runtime/state.json.
Targets and feedback use the version 1 file protocol documented in protocol.md.
No robot driver or network control connection is opened by this frontend.
"""
from pathlib import Path
import datetime
import json
import math
import os
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

import numpy as np
import qt
import slicer
import vtk

PROJECT = Path(__file__).resolve().parents[1]
RUNTIME = PROJECT / "runtime"
OUTPUT = PROJECT / "output"
LINK_ORDER = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
              "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"]
BASE_TO_RAS_MM = np.eye(4)


def atomic_json(path, value):
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(path))


def rotation_rpy_degrees(roll, pitch, yaw):
    r, p, y = np.deg2rad([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def quaternion_from_rotation(rotation):
    q = [0.0] * 4
    vtk.vtkMath.Matrix3x3ToQuaternion(rotation.tolist(), q)
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def rotation_from_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError("Feedback contains an invalid quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_matrix(pose):
    matrix = np.eye(4)
    position = np.asarray(pose["position_m"], dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Feedback contains an invalid position")
    matrix[:3, :3] = rotation_from_quaternion(pose["orientation_xyzw"])
    matrix[:3, 3] = position * 1000.0
    return BASE_TO_RAS_MM @ matrix


def vtk_matrix(array):
    result = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            result.SetElement(i, j, float(array[i, j]))
    return result


def polyline(points, radius=5.0):
    line = vtk.vtkPolyLine()
    vertices = vtk.vtkPoints()
    line.GetPointIds().SetNumberOfIds(len(points))
    for index, point in enumerate(points):
        vertices.InsertNextPoint(*[float(c) for c in point])
        line.GetPointIds().SetId(index, index)
    cells = vtk.vtkCellArray()
    cells.InsertNextCell(line)
    data = vtk.vtkPolyData()
    data.SetPoints(vertices)
    data.SetLines(cells)
    tube = vtk.vtkTubeFilter()
    tube.SetInputData(data)
    tube.SetRadius(radius)
    tube.SetNumberOfSides(16)
    tube.CappingOn()
    tube.Update()
    result = vtk.vtkPolyData()
    result.DeepCopy(tube.GetOutput())
    return result


def ball(radius=20.0):
    source = vtk.vtkSphereSource()
    source.SetRadius(radius)
    source.SetThetaResolution(24)
    source.SetPhiResolution(16)
    source.Update()
    return source.GetOutput()


def model(name, data, color, opacity=1.0):
    node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", name)
    node.SetAndObservePolyData(data)
    node.CreateDefaultDisplayNodes()
    display = node.GetDisplayNode()
    display.SetColor(*color)
    display.SetOpacity(opacity)
    display.SetBackfaceCulling(False)
    return node


def collada_polydata(filename):
    """Read the matrix/polylist/triangles subset in the pinned UR visual DAE.

    Authored scene matrices are applied first; output remains in metres, before
    the URDF visual-origin transform. Unexpected transforms fail explicitly.
    """
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(str(filename)).getroot()
    geometries = {}
    for geometry in root.findall("c:library_geometries/c:geometry", ns):
        mesh = geometry.find("c:mesh", ns)
        sources = {}
        for source in mesh.findall("c:source", ns):
            values = np.fromstring(source.find("c:float_array", ns).text, sep=" ")
            stride = int(source.find("c:technique_common/c:accessor", ns).get("stride", "1"))
            sources[source.get("id")] = values.reshape((-1, stride))
        vertices = {}
        for vertex in mesh.findall("c:vertices", ns):
            for item in vertex.findall("c:input", ns):
                if item.get("semantic") == "POSITION":
                    vertices[vertex.get("id")] = item.get("source").lstrip("#")
        parts = []
        for primitive in list(mesh):
            primitive_type = primitive.tag.split("}")[-1]
            if primitive_type not in ("polylist", "triangles"):
                continue
            inputs = primitive.findall("c:input", ns)
            stride = 1 + max(int(item.get("offset", "0")) for item in inputs)
            vertex_input = next(item for item in inputs if item.get("semantic") == "VERTEX")
            offset = int(vertex_input.get("offset", "0"))
            positions = sources[vertices[vertex_input.get("source").lstrip("#")]][:, :3]
            indices = np.fromstring(primitive.find("c:p", ns).text, sep=" ", dtype=np.int64).reshape((-1, stride))[:, offset]
            counts = np.full(int(primitive.get("count")), 3, dtype=int) if primitive_type == "triangles" else np.fromstring(primitive.find("c:vcount", ns).text, sep=" ", dtype=int)
            data = vtk.vtkPolyData()
            points = vtk.vtkPoints()
            for point in positions:
                points.InsertNextPoint(*point)
            cells = vtk.vtkCellArray()
            index = 0
            for count in counts:
                cells.InsertNextCell(int(count))
                for vertex_id in indices[index:index+count]:
                    cells.InsertCellPoint(int(vertex_id))
                index += int(count)
            data.SetPoints(points)
            data.SetPolys(cells)
            parts.append(data)
        geometries[geometry.get("id")] = parts
    append = vtk.vtkAppendPolyData()

    def visit(node, parent):
        local = np.eye(4)
        for item in list(node):
            tag = item.tag.split("}")[-1]
            if tag == "matrix":
                local = local @ np.fromstring(item.text, sep=" ").reshape((4, 4))
            elif tag in ("rotate", "translate", "scale", "lookat", "skew"):
                raise ValueError("Unsupported transform in pinned UR DAE: " + tag)
        world = parent @ local
        for instance in node.findall("c:instance_geometry", ns):
            for part in geometries[instance.get("url").lstrip("#")]:
                transform = vtk.vtkTransform()
                transform.SetMatrix(vtk_matrix(world))
                filtered = vtk.vtkTransformPolyDataFilter()
                filtered.SetTransform(transform)
                filtered.SetInputData(part)
                filtered.Update()
                append.AddInputData(filtered.GetOutput())
        for child in node.findall("c:node", ns):
            visit(child, world)

    for visual_scene in root.findall("c:library_visual_scenes/c:visual_scene", ns):
        for node in visual_scene.findall("c:node", ns):
            visit(node, np.eye(4))
    append.Update()
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(append.GetOutputPort())
    normals.SetFeatureAngle(45)
    normals.Update()
    result = vtk.vtkPolyData()
    result.DeepCopy(normals.GetOutput())
    if result.GetNumberOfPoints() == 0:
        raise ValueError("No geometry in UR visual asset: " + str(filename))
    return result


class UR12ePanel:
    def __init__(self):
        RUNTIME.mkdir(parents=True, exist_ok=True)
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.state = None
        self.feedback_error = None
        self.last_state_stamp = None
        self.last_successful_read = 0.0
        self.last_request = None
        self.planned_request = None
        self.recording = None
        self.recorded_frames = []
        self.last_frame = 0.0
        self.last_frame_monotonic = 0.0
        self.updating_target = False
        self.robot_models = {}
        self.mesh_transforms = {}
        self.fallback_models = []
        self.joint_models = []
        self.last_mesh_check = 0.0
        self.last_heartbeat = 0.0
        self.build_scene()
        self.build_panel()
        self.update_target()
        self.timer = qt.QTimer()
        self.timer.setInterval(100)
        self.timer.connect("timeout()", self.poll)
        self.timer.start()
        self.apply_camera()
        slicer.util.mainWindow().setWindowTitle("UR12e + 3D Slicer | SIMULATION")
        self.session_info = {
            "pid": os.getpid(), "started_at": time.time(), "slicer_version": slicer.app.applicationVersion,
            "simulation_only": True, "base_to_ras_mm": BASE_TO_RAS_MM.tolist(),
            "tool_tip_offset_m": [0, 0, 0.12], "frontend_script": str(Path(__file__).resolve())}
        self.session_info["heartbeat"] = time.time()
        atomic_json(RUNTIME / "slicer_session.json", self.session_info)

    def build_scene(self):
        slicer.mrmlScene.Clear(0)
        lm = slicer.app.layoutManager()
        lm.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutOneUp3DView)
        view = lm.threeDWidget(0).mrmlViewNode()
        view.SetBackgroundColor(0.025, 0.04, 0.07)
        view.SetBackgroundColor2(0.08, 0.11, 0.16)
        view.SetAxisLabelsVisible(False)
        view.SetBoxVisible(False)
        self.target_transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "TargetTCP_RAS_mm")
        self.tcp_transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "FeedbackTCP_RAS_mm")
        self.target_marker = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", "Target_tip")
        self.target_marker.AddControlPoint(vtk.vtkVector3d(450, -350, 450), "Target tip")
        self.target_marker.GetDisplayNode().SetSelectedColor(1.0, 0.7, 0.15)
        self.target_marker.GetDisplayNode().SetGlyphScale(1.6)
        self.target_marker.GetDisplayNode().SetTextScale(1.5)
        self.target_marker.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent, self.target_dragged)
        for prefix, transform, opacity in [("Target", self.target_transform, 0.9), ("Actual", self.tcp_transform, 1.0)]:
            for axis, color in enumerate([(1, .25, .22), (.25, 1, .4), (.25, .65, 1)]):
                end = [0, 0, 0]
                end[axis] = 100
                node = model(prefix + "_axis_" + "XYZ"[axis], polyline([[0, 0, 0], end], 3), color, opacity)
                node.SetAndObserveTransformNodeID(transform.GetID())
                if prefix == "Actual":
                    node.GetDisplayNode().SetVisibility(False)
                    self.robot_models[node.GetName()] = node
        self.tool = model("Actual_simplified_tool", polyline([[0, 0, 0], [0, 0, 120]], 7), (.1, .9, .95))
        self.tool_transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "FeedbackTool0_RAS_mm")
        self.tool.SetAndObserveTransformNodeID(self.tool_transform.GetID())
        self.tool.GetDisplayNode().SetVisibility(False)
        self.plan_path = model("MoveIt_planned_tip_path", polyline([[0, 0, 0], [0, 0, .001]], 3), (1, .72, .18), .85)
        self.plan_path.GetDisplayNode().SetVisibility(False)
        # A neutral floor provides scale; it is not an obstacle in the planner.
        cube = vtk.vtkCubeSource()
        cube.SetCenter(0, 0, -30)
        cube.SetXLength(2200)
        cube.SetYLength(1800)
        cube.SetZLength(20)
        cube.Update()
        model("Display_floor_not_collision_geometry", cube.GetOutput(), (.25, .3, .35), .23)
        for i in range(len(LINK_ORDER)-1):
            node = model("Feedback_link_" + str(i), polyline([[0, 0, 0], [0, 0, .001]], 42 if i < 4 else 30), (.56, .7, .78))
            node.GetDisplayNode().SetVisibility(False)
            self.fallback_models.append(node)
        for link in LINK_ORDER:
            node = model("Feedback_joint_" + link, ball(52 if link in LINK_ORDER[:4] else 36), (.13, .44, .67))
            transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", link + "_RAS_mm")
            node.SetAndObserveTransformNodeID(transform.GetID())
            node.GetDisplayNode().SetVisibility(False)
            self.joint_models.append((link, node, transform))

    def build_panel(self):
        previous = slicer.util.mainWindow().findChild(qt.QDockWidget, "UR12eSimulationDock")
        if previous:
            slicer.util.mainWindow().removeDockWidget(previous)
            previous.deleteLater()
        self.dock = qt.QDockWidget("UR12e 仿真联动", slicer.util.mainWindow())
        self.dock.setObjectName("UR12eSimulationDock")
        self.dock.setMinimumWidth(360)
        body = qt.QWidget()
        layout = qt.QVBoxLayout(body)
        title = qt.QLabel("<b>UR12e · Slicer · MoveIt 2</b><br/>静态目标 / 简化针状工具 / 仿真反馈")
        title.setWordWrap(True)
        layout.addWidget(title)
        note = qt.QLabel("目标是针尖；针尖位于 tool0 的 +Z 方向 120 mm。\n位置使用 Slicer RAS（mm）；朝向为固定轴 XYZ（°）。\n当前 RAS 与机器人 base 同轴同原点，仅为已知模拟关系。")
        note.setWordWrap(True)
        layout.addWidget(note)
        form = qt.QFormLayout()
        self.position = []
        self.angles = []
        for text, default in [("R / X", 450), ("A / Y", -350), ("S / Z", 450)]:
            spin = qt.QDoubleSpinBox()
            spin.setRange(-2000, 2000)
            spin.setDecimals(1)
            spin.setSingleStep(10)
            spin.setSuffix(" mm")
            spin.setValue(default)
            spin.connect("valueChanged(double)", self.update_target)
            self.position.append(spin)
            form.addRow(text, spin)
        for text, default in [("Roll / X", 180), ("Pitch / Y", 0), ("Yaw / Z", 0)]:
            spin = qt.QDoubleSpinBox()
            spin.setRange(-180, 180)
            spin.setDecimals(1)
            spin.setSingleStep(5)
            spin.setSuffix(" °")
            spin.setValue(default)
            spin.connect("valueChanged(double)", self.update_target)
            self.angles.append(spin)
            form.addRow(text, spin)
        layout.addLayout(form)
        copy_button = qt.QPushButton("以实际针尖为目标（上移 30 mm）")
        copy_button.connect("clicked()", self.copy_feedback_target)
        layout.addWidget(copy_button)
        buttons = qt.QHBoxLayout()
        self.submit_buttons = []
        for label, command in [("规划", "plan"), ("执行已规划路径", "execute"), ("规划并执行", "plan_execute")]:
            button = qt.QPushButton(label)
            button.connect("clicked()", lambda command=command: self.send(command))
            if command == "execute":
                self.execute_button = button
            button.setEnabled(False)
            self.submit_buttons.append((command, button))
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.status_label = qt.QLabel("等待 ROS 2 仿真反馈…")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(65)
        layout.addWidget(self.status_label)
        self.error_label = qt.QLabel("针尖位置误差：—\n朝向误差：—")
        self.error_label.setWordWrap(True)
        layout.addWidget(self.error_label)
        self.pose_label = qt.QLabel("实际针尖位置：—")
        self.pose_label.setWordWrap(True)
        layout.addWidget(self.pose_label)
        self.plan_label = qt.QLabel("规划：—")
        self.plan_label.setWordWrap(True)
        layout.addWidget(self.plan_label)
        self.model_label = qt.QLabel("显示：等待实际反馈；连杆几何占位")
        self.model_label.setWordWrap(True)
        layout.addWidget(self.model_label)
        capture = qt.QHBoxLayout()
        for label, callback in [("保存场景与结果", self.save), ("截图", self.capture), ("重置视角", self.apply_camera)]:
            button = qt.QPushButton(label)
            button.connect("clicked()", callback)
            capture.addWidget(button)
        layout.addLayout(capture)
        self.record_button = qt.QPushButton("开始记录演示帧")
        self.record_button.connect("clicked()", self.toggle_recording)
        layout.addWidget(self.record_button)
        self.output_label = qt.QLabel("")
        self.output_label.setWordWrap(True)
        layout.addWidget(self.output_label)
        layout.addStretch(1)
        self.dock.setWidget(body)
        slicer.util.mainWindow().addDockWidget(qt.Qt.RightDockWidgetArea, self.dock)
        self.dock.show()
        slicer.util.mainWindow().showNormal()
        slicer.util.mainWindow().resize(1600, 960)
        module_panel = slicer.util.mainWindow().findChild(qt.QDockWidget, "PanelDockWidget")
        if module_panel:
            module_panel.hide()

    def apply_camera(self):
        camera = slicer.app.layoutManager().threeDWidget(0).threeDView().renderWindow().GetRenderers().GetFirstRenderer().GetActiveCamera()
        camera.SetPosition(2100, -2400, 1700)
        camera.SetFocalPoint(0, -150, 450)
        camera.SetViewUp(0, 0, 1)
        camera.SetParallelProjection(True)
        camera.SetParallelScale(1000)
        slicer.app.layoutManager().threeDWidget(0).threeDView().resetFocalPoint()
        camera.SetFocalPoint(0, -150, 450)
        camera.SetParallelScale(1000)
        slicer.app.layoutManager().threeDWidget(0).threeDView().scheduleRender()

    def current_target(self):
        ras = np.array([float(spin.value) for spin in self.position])
        rotation_ras = rotation_rpy_degrees(*[float(spin.value) for spin in self.angles])
        ras_to_base = np.linalg.inv(BASE_TO_RAS_MM)
        base = (ras_to_base @ np.r_[ras, 1])[:3] / 1000.0
        rotation_base = ras_to_base[:3, :3] @ rotation_ras
        return {"frame_id": "base", "position_m": base.tolist(),
                "orientation_xyzw": quaternion_from_rotation(rotation_base)}

    def update_target(self, *args):
        if self.updating_target or not hasattr(self, "angles") or len(self.angles) != 3:
            return
        self.updating_target = True
        try:
            target = self.current_target()
            self.target_transform.SetMatrixTransformToParent(vtk_matrix(pose_matrix(target)))
            point = (BASE_TO_RAS_MM @ np.r_[np.array(target["position_m"])*1000, 1])[:3]
            self.target_marker.SetNthControlPointPosition(0, *point)
            if hasattr(self, "execute_button"):
                self.execute_button.setEnabled(self.can_execute())
            if self.state:
                self.show_errors(self.state)
        finally:
            self.updating_target = False

    def target_dragged(self, *args):
        if self.updating_target or not hasattr(self, "position"):
            return
        self.updating_target = True
        xyz = self.target_marker.GetNthControlPointPositionWorld(0)
        for spin, value in zip(self.position, xyz):
            spin.setValue(value)
        self.updating_target = False
        self.update_target()

    def same_target(self, pose):
        if not pose:
            return False
        target = self.current_target()
        return np.allclose(target["position_m"], pose["position_m"], atol=1e-8) and abs(np.dot(target["orientation_xyzw"], pose["orientation_xyzw"])) > 1-1e-9

    def can_execute(self):
        if self.feedback_error or not self.planned_request or not self.state:
            return False
        return self.same_target(self.planned_request) and self.state.get("status") == "planned" and self.state.get("ready", False) and time.time()-float(self.state.get("timestamp", 0)) < 5

    def copy_feedback_target(self):
        if self.feedback_error or not self.state or not self.state.get("tcp_pose") or not self.state.get("ready", False) or time.time()-float(self.state.get("timestamp", 0)) > 5:
            self.status_label.setText("尚无新鲜的针尖反馈，无法复制实际位姿。")
            return
        matrix = pose_matrix(self.state["tcp_pose"])
        rotation = matrix[:3, :3]
        pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
        if abs(math.cos(pitch)) > 1e-6:
            roll = math.atan2(rotation[2, 1], rotation[2, 2])
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        else:
            roll = 0.0
            yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
        self.updating_target = True
        for spin, value in zip(self.position, matrix[:3, 3] + [0, 0, 30]):
            spin.setValue(float(value))
        for spin, value in zip(self.angles, np.rad2deg([roll, pitch, yaw])):
            spin.setValue(float(value))
        self.updating_target = False
        self.update_target()

    def send(self, command):
        if self.feedback_error or not self.state or not self.state.get("ready", False) or time.time()-float(self.state.get("timestamp", 0)) > 5:
            self.status_label.setText("ROS 2 仿真尚未就绪；收到新鲜的实际状态后再提交目标。")
            return None
        if self.state.get("status") in ("planning", "executing"):
            self.status_label.setText("当前操作尚未结束，请等待规划或执行完成。")
            return None
        if command == "execute" and not self.can_execute():
            self.status_label.setText("请先为当前目标规划，收到有效计划后再执行。")
            return None
        target = self.current_target()
        request = dict(target, schema_version=1, request_id=uuid.uuid4().hex, command=command, timestamp=time.time())
        if command == "execute":
            request["plan_request_id"] = self.planned_request["request_id"]
        atomic_json(RUNTIME / "target.json", request)
        self.last_request = request
        self.execute_button.setEnabled(False)
        for _, button in self.submit_buttons:
            button.setEnabled(False)
        self.status_label.setText("已发送：" + {"plan": "规划", "execute": "执行已规划路径", "plan_execute": "规划并执行"}[command] + "；等待机器人侧处理。")
        return request

    def poll(self):
        try:
            if time.time()-self.last_heartbeat > 1:
                self.session_info["heartbeat"] = time.time()
                atomic_json(RUNTIME / "slicer_session.json", self.session_info)
                self.last_heartbeat = time.time()
            path = RUNTIME / "state.json"
            if path.exists():
                stamp = path.stat().st_mtime_ns
                if stamp != self.last_state_stamp:
                    state = json.loads(path.read_text(encoding="utf-8"))
                    if state.get("schema_version") != 1:
                        raise ValueError("Unsupported feedback schema version")
                    if state.get("source") != "ros2_mock_hardware":
                        raise ValueError("This simulation console accepts ros2_mock_hardware feedback only")
                    self.feedback_error = None
                    self.state = state
                    self.last_state_stamp = stamp
                    self.last_successful_read = time.time()
                    self.update_feedback(state)
            if self.state and time.time()-float(self.state.get("timestamp", 0)) > 5:
                self.status_label.setText("反馈已中断；画面保留最后一次实际状态，当前不能据此确认到位。")
                for _, button in self.submit_buttons:
                    button.setEnabled(False)
                self.set_robot_opacity(.2)
                self.show_errors(self.state)
            if time.time()-self.last_mesh_check > 5:
                self.load_visual_meshes()
                self.last_mesh_check = time.time()
            if self.recording and time.monotonic()-self.last_frame_monotonic >= .2:
                self.capture_recording_frame()
        except Exception as error:
            self.feedback_error = str(error)
            self.status_label.setText("反馈无效，控制已停用；保留末次实际姿态。\n读取失败：" + str(error))
            for _, button in self.submit_buttons:
                button.setEnabled(False)
            self.set_robot_opacity(.2)
            self.model_label.setText("反馈无效：保留末次实际姿态")
            if self.state:
                self.show_errors(self.state)
            (RUNTIME / "slicer_feedback_error.txt").write_text(traceback.format_exc(), encoding="utf-8")

    def update_feedback(self, state):
        links = state.get("link_transforms", {})
        for link, node, transform in self.joint_models:
            available = link in links
            node.GetDisplayNode().SetVisibility(available and not self.mesh_transforms)
            if available:
                transform.SetMatrixTransformToParent(vtk_matrix(pose_matrix(links[link])))
        for i, node in enumerate(self.fallback_models):
            a, b = LINK_ORDER[i:i+2]
            available = a in links and b in links
            node.GetDisplayNode().SetVisibility(available and not self.mesh_transforms)
            if available:
                node.SetAndObservePolyData(polyline([pose_matrix(links[a])[:3, 3], pose_matrix(links[b])[:3, 3]], 42 if i < 4 else 30))
        for link, entries in self.mesh_transforms.items():
            for mesh_node, transform in entries:
                mesh_node.GetDisplayNode().SetVisibility(link in links)
                if link in links:
                    transform.SetMatrixTransformToParent(vtk_matrix(pose_matrix(links[link])))
        tcp = state.get("tcp_pose")
        for node in self.robot_models.values():
            node.GetDisplayNode().SetVisibility(tcp is not None)
        if tcp:
            self.tcp_transform.SetMatrixTransformToParent(vtk_matrix(pose_matrix(tcp)))
        self.tool.GetDisplayNode().SetVisibility("tool0" in links)
        if "tool0" in links:
            self.tool_transform.SetMatrixTransformToParent(vtk_matrix(pose_matrix(links["tool0"])))
            offset = np.array(state.get("tool", {}).get("tip_offset_m", [0, 0, .12])) * 1000
            self.tool.SetAndObservePolyData(polyline([[0, 0, 0], offset], 7))
        plan = state.get("plan", {})
        path = plan.get("tcp_path_m", [])
        self.plan_path.GetDisplayNode().SetVisibility(len(path) >= 2 and plan.get("status") in ["planned", "valid", "succeeded"])
        if len(path) >= 2:
            self.plan_path.SetAndObservePolyData(polyline([(BASE_TO_RAS_MM @ np.r_[np.array(p)*1000, 1])[:3] for p in path], 3))
        labels = {"idle":"就绪", "waiting_for_robot":"等待机器人状态", "planning":"规划中", "planned":"规划成功，等待执行", "executing":"仿真执行中", "succeeded":"仿真执行完成", "error":"操作失败"}
        status = state.get("status", "unknown")
        fixture = state.get("source") == "test_fixture"
        self.status_label.setText(("界面测试数据（非 ROS 反馈）\n" if fixture else "") + labels.get(status, status) + "\n" + str(state.get("message", "")))
        self.plan_label.setText("规划：{}；轨迹点 {}".format(plan.get("status", "—"), plan.get("point_count", "—")))
        if status == "planned" and state.get("target_pose"):
            self.planned_request = dict(state["target_pose"], request_id=plan.get("request_id") or state.get("request_id"))
        self.execute_button.setEnabled(self.can_execute())
        for command, button in self.submit_buttons:
            if command != "execute":
                button.setEnabled(bool(state.get("ready", False)) and status not in ("planning", "executing"))
        self.set_robot_opacity(1.0 if state.get("ready", False) else .2)
        self.show_errors(state)
        if not self.mesh_transforms and links:
            self.model_label.setText("显示：实际 TF 反馈 / 连杆几何占位")
        elif self.mesh_transforms and links:
            self.model_label.setText("显示：UR 官方模型 / " + ("界面测试位姿" if fixture else "实际 TF 反馈"))
        annotation = slicer.app.layoutManager().threeDWidget(0).threeDView().cornerAnnotation()
        annotation.SetText(2, ("FRONTEND TEST FIXTURE - NOT ROS FEEDBACK\n" if fixture else "UR12e | SIMULATION\n") + "Target: gold / Tool: cyan\n" + labels.get(status, status))

    def set_robot_opacity(self, value):
        nodes = [node for _, node, _ in self.joint_models] + self.fallback_models + [self.tool] + list(self.robot_models.values())
        nodes += [node for entries in self.mesh_transforms.values() for node, _ in entries]
        for node in nodes:
            node.GetDisplayNode().SetOpacity(value)

    def show_errors(self, state):
        tcp = state.get("tcp_pose")
        if not tcp:
            self.error_label.setText("针尖位置误差：—\n朝向误差：—")
            self.pose_label.setText("实际针尖位置：暂无反馈")
            return
        target = self.current_target()
        position_error = np.linalg.norm(np.array(tcp["position_m"])-target["position_m"])*1000
        qa = np.asarray(tcp["orientation_xyzw"], dtype=float)
        qb = np.asarray(target["orientation_xyzw"], dtype=float)
        cos_half = np.clip(abs(np.dot(qa/np.linalg.norm(qa), qb/np.linalg.norm(qb))), 0, 1)
        degrees = math.degrees(2*math.acos(float(cos_half)))
        stale = not state.get("ready", False) or time.time()-float(state.get("timestamp", 0)) > 5
        invalid_label = "末次反馈（当前读取失败）\n" if self.feedback_error else ("末次反馈（已过期）\n" if stale else "")
        self.error_label.setText(invalid_label + "当前目标的针尖位置误差：{:.3f} mm\n朝向误差：{:.3f} °".format(position_error, degrees))
        xyz = (BASE_TO_RAS_MM @ np.r_[np.array(tcp["position_m"])*1000, 1])[:3]
        self.pose_label.setText(("末次针尖 RAS（反馈无效）：" if self.feedback_error or stale else "实际针尖 RAS：") + "{:.1f}, {:.1f}, {:.1f} mm".format(*xyz))

    def load_visual_meshes(self):
        if self.mesh_transforms:
            return
        manifest = PROJECT / "assets" / "visual_model.json"
        if not manifest.exists():
            return
        metadata = json.loads(manifest.read_text(encoding="utf-8"))
        pending = []
        try:
            for entry in metadata["links"]:
                filename = Path(entry["file"])
                if not filename.is_absolute():
                    filename = PROJECT / filename
                node = model("UR visual", collada_polydata(filename), entry.get("color", [.65, .72, .77])[:3]) if filename.suffix.lower() == ".dae" else slicer.util.loadModel(str(filename))
                if not node:
                    raise RuntimeError("Unable to load robot visual: " + str(filename))
                pending.append(node)
                node.SetName("UR12e_visual_" + entry["link"])
                # Mesh local origin is expressed in metres in URDF, Slicer uses mm.
                rotation = rotation_rpy_degrees(*np.rad2deg(entry.get("origin_rpy_rad", [0, 0, 0])))
                origin = np.eye(4)
                origin[:3, :3] = rotation @ np.diag(np.array(entry.get("scale", [1, 1, 1]))*1000)
                origin[:3, 3] = np.array(entry.get("origin_xyz_m", [0, 0, 0]))*1000
                transform_filter = vtk.vtkTransformPolyDataFilter()
                transform = vtk.vtkTransform()
                transform.SetMatrix(vtk_matrix(origin))
                transform_filter.SetTransform(transform)
                transform_filter.SetInputData(node.GetPolyData())
                transform_filter.Update()
                data = vtk.vtkPolyData()
                data.DeepCopy(transform_filter.GetOutput())
                node.SetAndObservePolyData(data)
                color = entry.get("color", [.65, .72, .77])[:3]
                node.GetDisplayNode().SetColor(*color)
                transform_node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", entry["link"] + "_visual_TF")
                node.SetAndObserveTransformNodeID(transform_node.GetID())
                node.GetDisplayNode().SetVisibility(False)
                self.mesh_transforms.setdefault(entry["link"], []).append((node, transform_node))
            self.model_label.setText("UR 官方模型已加载；等待 TF 显示" if not self.state else "显示：UR 官方网格 / 实际 TF 反馈")
            if self.state:
                self.update_feedback(self.state)
        except Exception:
            for node in pending:
                slicer.mrmlScene.RemoveNode(node)
            self.mesh_transforms = {}
            raise

    def capture(self, destination=None):
        if destination is None or isinstance(destination, bool):
            destination = OUTPUT / ("slicer_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".png")
        destination = Path(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        slicer.app.layoutManager().threeDWidget(0).threeDView().renderWindow().Render()
        slicer.app.processEvents()
        slicer.util.mainWindow().grab().save(str(destination))
        self.output_label.setText("已保存：" + str(destination))
        return str(destination)

    def save(self):
        folder = OUTPUT / ("session_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        folder.mkdir(parents=True, exist_ok=True)
        saved = slicer.util.saveScene(str(folder / "UR12e_simulation.mrb"))
        record = {"scene_saved": bool(saved), "saved_at": time.time(), "simulation_only": True,
                  "base_to_ras_mm": BASE_TO_RAS_MM.tolist(), "target": self.current_target(),
                  "last_request": self.last_request, "actual_feedback": self.state,
                  "feedback_age_s": None if not self.state else time.time()-float(self.state.get("timestamp", 0)),
                  "robot_display": "official_mesh" if self.mesh_transforms else "link_geometry_placeholder"}
        atomic_json(folder / "results.json", record)
        self.capture(folder / "slicer.png")
        self.output_label.setText("已保存场景、截图和实际反馈：\n" + str(folder))
        return str(folder)

    def toggle_recording(self):
        if self.recording:
            atomic_json(self.recording / "frames.json", {"frame_rate": 5, "frames": self.recorded_frames,
                        "simulation_only": True, "capture": "Slicer main window + feedback snapshot",
                        "image_storage": "Real window grab, width at most 1600 px, lossless PNG with low compression",
                        "timestamps": "Windows wall clock at each original window grab; scheduling uses monotonic clock"})
            self.output_label.setText("演示帧已保存：\n" + str(self.recording))
            self.recording = None
            self.record_button.setText("开始记录演示帧")
        else:
            self.recording = OUTPUT / ("recording_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            self.recording.mkdir(parents=True, exist_ok=True)
            self.recorded_frames = []
            self.record_button.setText("结束并保存演示帧")
            self.last_frame = 0
            self.last_frame_monotonic = 0

    def capture_recording_frame(self):
        self.last_frame = time.time()
        self.last_frame_monotonic = time.monotonic()
        started = time.perf_counter()
        name = "frame_{:05d}.png".format(len(self.recorded_frames))
        pixmap = slicer.util.mainWindow().grab()
        source_size = [pixmap.width(), pixmap.height()]
        if pixmap.width() > 1600:
            pixmap = pixmap.scaledToWidth(1600, qt.Qt.SmoothTransformation)
        # Qt PNG quality=100 selects minimal compression while retaining pixels.
        # Default compression of a 3200 px window blocked the GUI for ~1 s/frame.
        if not pixmap.save(str(self.recording / name), "PNG", 100):
            raise RuntimeError("Unable to save the real Slicer recording frame")
        self.recorded_frames.append({"file": name, "timestamp": self.last_frame, "feedback": self.state,
                                     "source_window_size": source_size, "saved_size": [pixmap.width(), pixmap.height()],
                                     "capture_duration_s": time.perf_counter()-started})

    def stop(self):
        self.timer.stop()
        if self.recording:
            self.toggle_recording()
        slicer.util.mainWindow().removeDockWidget(self.dock)
        self.dock.deleteLater()


def start():
    if hasattr(slicer, "ur12eSimulation"):
        slicer.ur12eSimulation.stop()
    slicer.ur12eSimulation = UR12ePanel()
    return slicer.ur12eSimulation
