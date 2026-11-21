"""Static head models and needle clearance replay in the live UR12e panel."""
import json
import math
from pathlib import Path
import sys
import uuid

import numpy as np
import qt
import slicer
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

from ur12e_panel import BASE_TO_RAS_MM, PROJECT, CONTROL_RUNTIME, atomic_json, ball, model, polyline, pose_matrix, vtk_matrix

sys.path.insert(0, str(PROJECT / "ros2_ws/src/ur12e_slicer_bridge"))
from ur12e_slicer_bridge.anatomy_geometry import anatomy_archive


LABELS = {"brain": "脑模", "skull": "颅骨"}
DEMO_OFFSET_M = [0.0, 0.35, 0.0]


def model_polydata(points, faces):
    vertices = vtk.vtkPoints()
    vertices.SetData(numpy_to_vtk(points, deep=True))
    cells = vtk.vtkCellArray()
    offsets = np.arange(0, faces.size + 1, 3, dtype=np.int64)
    cells.SetData(numpy_to_vtkIdTypeArray(offsets, deep=True),
                  numpy_to_vtkIdTypeArray(faces.astype(np.int64).ravel(), deep=True))
    data = vtk.vtkPolyData()
    data.SetPoints(vertices)
    data.SetPolys(cells)
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputData(data)
    normals.SplittingOff()
    normals.ConsistencyOn()
    normals.Update()
    return normals.GetOutput()


class AnatomyPanel:
    def __init__(self, panel, state):
        self.panel = panel
        self.head = None
        self.pending_revision = None
        self.locked_pose = None
        self.report_key = None
        self.report = None
        self.nodes = {}
        self.ghosts = {}
        self.preview_kind = None
        manifest, arrays = anatomy_archive(PROJECT / ".local/head_anatomy")
        self.asset_hash = manifest["geometry_sha256"]
        if self.asset_hash != state["asset_sha256"]:
            raise RuntimeError("Slicer head geometry differs from the bridge's source")
        scene = slicer.mrmlScene
        self.head_transform = scene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "Head_to_RAS_mm")
        with arrays:
            for name, color, opacity in (("brain", (.85, .46, .33), 1.0), ("skull", (.94, .90, .79), .24)):
                data = model_polydata(arrays[name + "_points_mm"], arrays[name + "_faces"])
                node = model("BrainWeb04_" + name, data, color, opacity)
                node.SetAndObserveTransformNodeID(self.head_transform.GetID())
                node.SetAttribute("Source", manifest["subject"])
                self.nodes[name] = node
        self.witness = model("Minimum_clearance_witness", ball(4.5), (1.0, .29, .21))
        self.witness_transform = scene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "Clearance_witness_RAS_mm")
        self.witness.SetAndObserveTransformNodeID(self.witness_transform.GetID())
        self.ghost_tool = model("Replay_needle", ball(3), (.15, .96, .82))
        self.target_axes = [node for node in slicer.util.getNodesByClass("vtkMRMLModelNode")
                            if node.GetTransformNodeID() == panel.target_transform.GetID()]
        for axis in self.target_axes:
            axis.GetDisplayNode().SetVisibility(False)
        self.hide_replay()
        self.mr_surface = self.load_original_mr()
        self.replay_timer = qt.QTimer()
        self.replay_timer.setInterval(90)
        self.replay_timer.connect("timeout()", self.advance)
        self.build_controls()
        self.update(state, {})
        self.frame_camera()

    def load_original_mr(self):
        import navigation_benchmark_scene as nav

        fixture = json.loads((PROJECT / "navigation_benchmark/fixtures/head_world_v1.json").read_text(encoding="utf-8"))
        nav._volume_surface({}, np.array(fixture["T_B_I"]))
        surface = slicer.util.getNode(nav.PREFIX + "MR_intensity_surface_NOT_validated_segmentation")
        surface.GetDisplayNode().SetVisibility(False)
        return surface

    def build_controls(self):
        layout = qt.QVBoxLayout(self.panel.anatomy_box)
        views = qt.QHBoxLayout()
        self.view = qt.QComboBox()
        self.view.addItems(["脑模 + 颅骨（BrainWeb）", "原 MR 头部显示"])
        self.view.connect("currentIndexChanged(int)", self.switch_view)
        views.addWidget(self.view, 1)
        focus = qt.QPushButton("聚焦")
        focus.connect("clicked()", self.frame_camera)
        views.addWidget(focus)
        layout.addLayout(views)
        self.scene_status = qt.QLabel()
        self.scene_status.setWordWrap(True)
        layout.addWidget(self.scene_status)
        pose = qt.QGridLayout()
        self.head_position = []
        for col, axis in enumerate(("X", "Y", "Z", "Yaw")):
            spin = qt.QDoubleSpinBox()
            spin.setRange(-180, 180) if axis == "Yaw" else spin.setRange(-2000, 2000)
            spin.setDecimals(1)
            spin.setSingleStep(5)
            spin.setToolTip("头部在 UR base 的位置（mm）或绕 +Z 角度（°）")
            self.head_position.append(spin)
            pose.addWidget(qt.QLabel(axis), 0, col)
            pose.addWidget(spin, 1, col)
        layout.addLayout(pose)
        actions = qt.QHBoxLayout()
        self.apply_button = qt.QPushButton("应用头位")
        self.apply_button.connect("clicked()", self.apply_pose)
        actions.addWidget(self.apply_button)
        target = qt.QPushButton("载入演示目标")
        target.connect("clicked()", self.demo_target)
        actions.addWidget(target)
        layout.addLayout(actions)
        playback = qt.QHBoxLayout()
        self.play = qt.QPushButton("播放")
        self.play.connect("clicked()", self.play_pause)
        self.closest_button = qt.QPushButton("最近姿态")
        self.closest_button.connect("clicked()", self.jump_closest)
        playback.addWidget(self.play)
        playback.addWidget(self.closest_button)
        layout.addLayout(playback)
        self.slider = qt.QSlider(qt.Qt.Horizontal)
        self.slider.connect("valueChanged(int)", self.show_frame)
        layout.addWidget(self.slider)
        self.distance_label = qt.QLabel("等待规划")
        self.distance_label.setWordWrap(True)
        layout.addWidget(self.distance_label)
        self.panel.anatomy_box.show()

    @property
    def legacy_visible(self):
        return self.view.currentIndex == 1

    def plan_enabled(self):
        return bool(self.head and self.head["ready"] and not self.legacy_visible
                    and self.pending_revision is None and self.head["asset_sha256"] == self.asset_hash)

    def switch_view(self, index):
        for node in self.nodes.values():
            node.GetDisplayNode().SetVisibility(index == 0)
        self.mr_surface.GetDisplayNode().SetVisibility(index == 1)
        self.hide_replay()
        self.frame_camera()
        self.panel.update_feedback(self.panel.state)
        self.refresh_controls()

    def frame_camera(self):
        if self.legacy_visible:
            bounds = [0.0] * 6
            self.mr_surface.GetRASBounds(bounds)
            focus = [(bounds[i] + bounds[i + 1]) / 2 for i in (0, 2, 4)]
        else:
            matrix = pose_matrix(self.head)
            focus = matrix[:3, 3] + [0, 0, 55]
        view = slicer.app.layoutManager().threeDWidget(0).threeDView()
        renderer = view.renderWindow().GetRenderers().GetFirstRenderer()
        camera = renderer.GetActiveCamera()
        camera.SetFocalPoint(*focus)
        camera.SetPosition(*(np.array(focus) + [-870, -490, 400]))
        camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(187 if self.legacy_visible else 300)
        renderer.ResetCameraClippingRange()
        view.scheduleRender()

    def apply_pose(self):
        if not self.head or self.legacy_visible or self.pending_revision is not None:
            return None
        if (CONTROL_RUNTIME / "validation.lock").exists() or self.panel.feedback_error:
            return None
        vals = [float(spin.value) for spin in self.head_position]
        yaw = math.radians(vals[-1])
        revision = uuid.uuid4().hex
        atomic_json(CONTROL_RUNTIME / "head_pose.json",
                    {"session": self.head["session"], "revision": revision, "position_m": [v / 1000 for v in vals[:3]],
                     "orientation_xyzw": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]})
        self.pending_revision = revision
        self.panel.planned_request = None
        self.panel.plan_path.GetDisplayNode().SetVisibility(False)
        self.hide_replay()
        self.refresh_controls()
        return revision

    def demo_target(self):
        tip = self.panel.state["tcp_pose"]
        target = dict(tip, position_m=(np.asarray(tip["position_m"]) + DEMO_OFFSET_M).tolist())
        self.set_target(target)

    def set_target(self, target):
        transform = pose_matrix(target)
        rot = transform[:3, :3]
        angles = np.rad2deg([math.atan2(rot[2, 1], rot[2, 2]), math.atan2(-rot[2, 0], math.hypot(rot[0, 0], rot[1, 0])),
                             math.atan2(rot[1, 0], rot[0, 0])])
        self.panel.updating_target = True
        try:
            for widget, value in zip(self.panel.position + self.panel.angles, [*transform[:3, 3], *angles]):
                widget.setValue(float(value))
        finally:
            self.panel.updating_target = False
        self.panel.update_target()

    def update(self, state, plan):
        if state["asset_sha256"] != self.asset_hash:
            raise RuntimeError("Reload Slicer after preparing a new head dataset")
        if self.head and (self.head["session"] != state["session"] or self.head["revision"] != state["revision"]):
            self.report_key = self.report = None
            self.hide_replay()
        self.head = state
        if self.pending_revision and (state["confirmed_revision"] == self.pending_revision or state.get("input_error")):
            self.pending_revision = None
        revision = state["revision"]
        if revision != self.locked_pose:
            self.locked_pose = revision
            self.head_transform.SetMatrixTransformToParent(vtk_matrix(pose_matrix(state)))
            q = state["orientation_xyzw"]
            vals = [*[v * 1000 for v in state["position_m"]], math.degrees(2 * math.atan2(q[2], q[3]))]
            for spin, value in zip(self.head_position, vals):
                spin.setValue(value)
        current = plan.get("head_revision") == revision and self.plan_enabled()
        filename = plan.get("head_report") if current else None
        if filename != self.report_key:
            self.report_key = filename
            self.report = None
            self.hide_replay()
            if filename:
                if Path(filename).name != filename or not filename.startswith("head_plan_"):
                    raise ValueError("Invalid head replay filename")
                result = json.loads((CONTROL_RUNTIME / filename).read_text(encoding="utf-8"))
                if result["head_revision"] != revision or result["asset_sha256"] != self.asset_hash:
                    raise ValueError("Head replay metadata differs from the active scene")
                self.report = result
                self.slider.setRange(0, len(result["frames"]) - 1)
                if result["passed"] and not self.legacy_visible:
                    self.update_witness(result["closest"]["witness_base_mm"])
        summary = plan.get("clearance") if current else None
        if summary and summary.get("passed"):
            if not self.ghost_tool.GetDisplayNode().GetVisibility():
                text = f"净距下界 {summary['certified_minimum_lower_mm']:.1f} mm"
                search = plan.get("head_search")
                if search:
                    labels = {"straight": "直达", "above_center": "头模上方绕行", "above": "头模上方绕行",
                              "left": "头模侧向绕行", "right": "头模侧向绕行", "ompl": "RRTConnect 回退"}
                    route = labels.get(search["selected_route"], "通过")
                    if search["strategy"] == "ompl":
                        route = "RRTConnect"
                    text = f"{route} · {1000 * search['operation_wall_s']:.0f} ms\n{text} · 全臂复核通过"
                self.distance_label.setText(text)
        elif summary and not summary.get("passed"):
            witness = summary["closest"]
            part = "针尖" if witness["part"] == "tip" else "针杆"
            self.distance_label.setText(f"拒绝 · {part}与{LABELS[witness['structure']]}的间隔未通过")
            if not self.legacy_visible:
                self.update_witness(witness["witness_base_mm"])
                if self.preview_kind != "replay":
                    self.draw_needle(witness, "rejected", (1.0, .29, .21))
        else:
            text = "等待规划" if state["ready"] else "等待头部场景确认"
            self.distance_label.setText("原 MR 头部查看" if self.legacy_visible else text)
            if not self.report:
                self.hide_replay()
        self.refresh_controls()

    def refresh_controls(self):
        ready = self.plan_enabled()
        if self.legacy_visible:
            status = "原 MR 影像与等值面"
        elif ready:
            status = "脑模、颅骨已在 MoveIt 确认 · 5 mm 净距"
        elif not self.head:
            status = "等待头部场景"
        else:
            status = self.head.get("input_error") or self.head.get("error") or "等待 MoveIt 同步头部场景"
        self.scene_status.setText(status)
        self.apply_button.setEnabled(bool(self.head and self.pending_revision is None and not self.legacy_visible))
        self.slider.setEnabled(ready and bool(self.report))
        self.closest_button.setEnabled(ready and bool(self.report))
        self.play.setEnabled(ready and bool(self.report))

    def hide_replay(self):
        for node, _ in self.ghosts.values():
            node.GetDisplayNode().SetVisibility(False)
        self.ghost_tool.GetDisplayNode().SetVisibility(False)
        self.preview_kind = None
        self.witness.GetDisplayNode().SetVisibility(False)
        if hasattr(self, "replay_timer"):
            self.replay_timer.stop()
        if hasattr(self, "play"):
            self.play.setText("播放")

    def update_witness(self, point):
        matrix = np.eye(4)
        matrix[:3, 3] = point
        self.witness_transform.SetMatrixTransformToParent(vtk_matrix(BASE_TO_RAS_MM @ matrix))
        self.witness.GetDisplayNode().SetVisibility(True)

    def draw_needle(self, frame, kind, color):
        points = [BASE_TO_RAS_MM @ np.r_[frame[key], 1] for key in ("tool_base_mm", "tip_base_mm")]
        self.ghost_tool.SetAndObservePolyData(polyline([point[:3] for point in points], 3.0))
        self.ghost_tool.GetDisplayNode().SetColor(*color)
        self.ghost_tool.GetDisplayNode().SetVisibility(True)
        self.preview_kind = kind

    def show_frame(self, index):
        if not self.report or not self.plan_enabled():
            return
        frame = self.report["frames"][index]
        if not self.ghosts:
            for link, entries in self.panel.mesh_transforms.items():
                if link == "base_link":
                    continue
                for idx, (original, _) in enumerate(entries):
                    node = model("Replay_" + link, original.GetPolyData(), (.41, .58, .73), .64)
                    transform = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "Replay_" + link + "_TF")
                    node.SetAndObserveTransformNodeID(transform.GetID())
                    self.ghosts[link, idx] = node, transform
        for (link, _), (node, transform) in self.ghosts.items():
            if link in frame["links_base_mm"]:
                matrix = BASE_TO_RAS_MM @ np.array(frame["links_base_mm"][link])
                transform.SetMatrixTransformToParent(vtk_matrix(matrix))
                node.GetDisplayNode().SetVisibility(True)
        self.draw_needle(frame, "replay", (.15, .96, .82))
        self.update_witness(frame["witness_base_mm"])
        part = "针尖" if frame["part"] == "tip" else "针杆"
        self.distance_label.setText(f"规划回放 {frame['time_s']:.2f} s · {part}/{LABELS[frame['structure']]}\n"
                                    f"当前净距下界 {max(0, frame['lower_mm']):.1f} mm")
        prefix = "BRAIN / SKULL   |   PLANNED POSE REPLAY"
        slicer.app.layoutManager().threeDWidget(0).threeDView().cornerAnnotation().SetText(2, prefix)

    def jump_closest(self):
        if not self.report:
            return
        self.replay_timer.stop()
        self.play.setText("播放")
        closest = self.report["closest"]
        index = next(i for i, frame in enumerate(self.report["frames"])
                     if frame["segment"] == closest["segment"] and frame["fraction"] == closest["fraction"])
        self.slider.setValue(index)
        self.show_frame(index)

    def play_pause(self):
        if not self.report:
            return
        if self.replay_timer.isActive():
            self.replay_timer.stop()
            self.play.setText("播放")
            return
        if self.slider.value == self.slider.maximum:
            self.slider.setValue(0)
        self.play.setText("暂停")
        self.replay_timer.start()

    def advance(self):
        if not self.plan_enabled() or self.slider.value == self.slider.maximum:
            self.replay_timer.stop()
            self.play.setText("播放")
            return
        self.slider.setValue(self.slider.value + 1)
