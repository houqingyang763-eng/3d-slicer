"""Capture the real Slicer view of the baseline, certified plan, and replay witness."""
from itertools import product
import sys

import numpy as np
import slicer

from ur12e_panel import BASE_TO_RAS_MM, PROJECT, model, polyline

sys.path.insert(0, str(PROJECT / "validation"))
from show_keepout_result import screenshot, text_actor


def corners(node):
    bounds = [0.0] * 6
    node.GetRASBounds(bounds)
    return list(product(bounds[:2], bounds[2:4], bounds[4:6]))


def frame_geometry(renderer, window, points):
    camera = renderer.GetActiveCamera()
    normal = np.array([-920.0, -370.0, 320.0])
    normal /= np.linalg.norm(normal)
    right = np.cross([0, 0, 1], normal)
    right /= np.linalg.norm(right)
    up = np.cross(normal, right)
    axes = np.stack([right, up, normal])
    projected = np.asarray(points) @ axes.T
    low, high = projected.min(axis=0), projected.max(axis=0)
    center = ((low + high) / 2) @ axes
    width, height = window.GetSize()
    horizontal, vertical = (high - low)[:2] / 2
    scale = max(178.0, vertical / .73, horizontal / (.81 * width / height))
    camera.SetFocalPoint(*center)
    camera.SetPosition(*(center + normal * 1400))
    camera.SetViewUp(*up)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(scale)
    renderer.ResetCameraClippingRange()


def capture(panel, folder, baseline):
    head = panel.anatomy
    if not head.report or not head.report["passed"] or not head.plan_enabled():
        raise RuntimeError("Expected a live Slicer replay of a certified plan")
    head.frame_camera()
    view = slicer.app.layoutManager().threeDWidget(0).threeDView()
    window = view.renderWindow()
    renderer = window.GetRenderers().GetFirstRenderer()
    annotation = view.cornerAnnotation()
    original_annotation = annotation.GetVisibility()
    old_path = panel.plan_path.GetDisplayNode()
    original_visibility = old_path.GetVisibility()
    original_opacity = head.nodes["brain"].GetDisplayNode().GetOpacity()
    robot = [node.GetDisplayNode() for entries in panel.mesh_transforms.values() for node, _ in entries]
    robot_opacity = [display.GetOpacity() for display in robot]
    nodes, actors = [], []
    gold, teal, white = (1, .69, .25), (.15, .95, .82), (.97, .97, 1)

    def path(points, name, color):
        ras = [(BASE_TO_RAS_MM @ np.r_[point, 1])[:3] for point in points]
        node = model(name, polyline(ras, 2.25), color)
        nodes.append(node)
        return node

    def display(text, color, x, y, size):
        actor = text_actor(text, color, x, y, size)
        renderer.AddActor2D(actor)
        actors.append(actor)

    def remove_actors():
        for actor in actors:
            renderer.RemoveActor2D(actor)
        actors.clear()

    try:
        head.hide_replay()
        old_path.SetVisibility(False)
        annotation.VisibilityOff()
        head.nodes["brain"].GetDisplayNode().SetOpacity(.58)
        certified = [frame["tip_base_mm"] for frame in head.report["frames"]]
        comparison_points = [(BASE_TO_RAS_MM @ np.r_[point, 1])[:3] for point in baseline + certified]
        comparison_points += corners(head.nodes["skull"])
        frame_geometry(renderer, window, comparison_points)
        for item in robot:
            item.SetOpacity(.48)
        control = path(baseline, "Baseline_MoveIt_tip_path", gold)
        path(certified, "Certified_MoveIt_tip_path", teal)
        display("UR12e  /  BRAIN AND SKULL", white, .035, .96, 22)
        display("SAME START AND NEEDLE-TIP GOAL", white, .035, .90, 15)
        display("BASELINE   intersects the protected anatomy", gold, .035, .128, 15)
        display("REPLAN   complete needle sweep certified", teal, .035, .073, 15)
        comparison = "01_anatomy_path_comparison.png"
        comparison_size = screenshot(window, folder / comparison)
        remove_actors()

        control.GetDisplayNode().SetVisibility(False)
        head.jump_closest()
        closest = head.report["closest"]
        displays = [head.ghost_tool.GetDisplayNode().GetVisibility(), head.witness.GetDisplayNode().GetVisibility()]
        if not all(displays):
            raise RuntimeError("Closest-pose replay did not display the needle and its witness")
        head.play_pause()
        playback_started = head.replay_timer.isActive()
        head.play_pause()
        if not playback_started or head.replay_timer.isActive():
            raise RuntimeError("Replay controls did not start and pause the Slicer timer")
        head.jump_closest()
        display("CLOSEST SAMPLED NEEDLE POSE", white, .035, .96, 22)
        display(f"{closest['structure'].upper()}   |   {closest['time_s']:.2f} s in plan preview", white, .035, .90, 15)
        bound = head.report["certified_minimum_lower_mm"]
        display(f"ENTIRE NEEDLE SWEEP   lower bound {bound:.2f} mm", teal, .035, .073, 15)
        replay = "02_closest_pose.png"
        replay_size = screenshot(window, folder / replay)
        scene_file = folder / "UR12e_Head_Anatomy.mrb"
        remove_actors()
        for item, opacity in zip(robot, robot_opacity):
            item.SetOpacity(opacity)
        head.hide_replay()
        overview_points = comparison_points + [p for entries in panel.mesh_transforms.values()
                                                for node, _ in entries for p in corners(node)]
        frame_geometry(renderer, window, overview_points)
        display("UR12e  /  STATIC BRAIN AND SKULL SCENE", white, .035, .96, 22)
        overview = "03_full_scene.png"
        overview_size = screenshot(window, folder / overview)
        remove_actors()
        control.GetDisplayNode().SetVisibility(True)
        frame_geometry(renderer, window, comparison_points)
        annotation.SetVisibility(original_annotation)
        if not slicer.util.saveScene(str(scene_file)) or not scene_file.is_file():
            raise RuntimeError("Failed to save the Slicer anatomy scene")
        return {"comparison_png": comparison, "replay_png": replay, "mrb": scene_file.name,
                "overview_png": overview, "overview_pixels": overview_size,
                "comparison_pixels": comparison_size, "replay_pixels": replay_size,
                "played_and_paused": True, "closest_frame": int(head.slider.value),
                "position_base_mm": closest["witness_base_mm"]}
    finally:
        remove_actors()
        for node in nodes:
            display_node = node.GetDisplayNode()
            slicer.mrmlScene.RemoveNode(node)
            if display_node and display_node.GetScene():
                slicer.mrmlScene.RemoveNode(display_node)
        head.nodes["brain"].GetDisplayNode().SetOpacity(original_opacity)
        for item, opacity in zip(robot, robot_opacity):
            item.SetOpacity(opacity)
        annotation.SetVisibility(original_annotation)
        old_path.SetVisibility(original_visibility)
        view.scheduleRender()


def capture_rejection(panel, destination, shaft_only):
    head = panel.anatomy
    if head.preview_kind != "rejected":
        raise RuntimeError("Slicer did not show the blocked nominal needle pose")
    view = slicer.app.layoutManager().threeDWidget(0).threeDView()
    window = view.renderWindow()
    renderer = window.GetRenderers().GetFirstRenderer()
    annotation = view.cornerAnnotation()
    visible = annotation.GetVisibility()
    brain = head.nodes["brain"].GetDisplayNode()
    opacity = brain.GetOpacity()
    title = "SHAFT INTERSECTS BRAIN" if shaft_only else "GOAL INSIDE BRAIN"
    detail = "Needle tip is outside both structures" if shaft_only else "The specified needle-tip goal lies inside the protected volume"
    actors = [
        text_actor(title, (.97, .97, 1), .035, .96, 22),
        text_actor(detail, (.97, .97, 1), .035, .90, 15),
        text_actor("REJECTED  /  Nominal needle pose", (1, .39, .30), .035, .075, 15),
    ]
    try:
        brain.SetOpacity(.48)
        annotation.VisibilityOff()
        frame_geometry(renderer, window, corners(head.nodes["skull"]) + corners(head.ghost_tool))
        for actor in actors:
            renderer.AddActor2D(actor)
        return screenshot(window, destination)
    finally:
        for actor in actors:
            renderer.RemoveActor2D(actor)
        brain.SetOpacity(opacity)
        annotation.SetVisibility(visible)
        head.frame_camera()
