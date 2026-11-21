"""Capture one Slicer camera view across the three recorded mock conditions."""
import json
from pathlib import Path

import numpy as np
import slicer
import vtk

from ur12e_panel import BASE_TO_RAS_MM, model, polyline


GOLD = (1.00, 0.73, 0.32)
TEAL = (0.22, 0.96, 0.84)
RED = (1.00, 0.34, 0.36)
WHITE = (0.95, 0.97, 1.00)


def ras(point_m):
    return (BASE_TO_RAS_MM @ np.r_[np.asarray(point_m) * 1000.0, 1.0])[:3]


def sphere(point_mm, radius_mm):
    source = vtk.vtkSphereSource()
    source.SetCenter(*point_mm)
    source.SetRadius(radius_mm)
    source.SetThetaResolution(64)
    source.SetPhiResolution(48)
    source.Update()
    return source.GetOutput()


def text_actor(text, color, x, y, size):
    actor = vtk.vtkTextActor()
    actor.SetInput(text)
    actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
    actor.SetPosition(x, y)
    style = actor.GetTextProperty()
    style.SetFontFamilyToArial()
    style.SetFontSize(size)
    style.SetColor(*color)
    style.SetVerticalJustificationToTop()
    return actor


def screenshot(window, path):
    window.Render()
    pixels = vtk.vtkWindowToImageFilter()
    pixels.SetInput(window)
    pixels.ReadFrontBufferOff()
    pixels.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetInputConnection(pixels.GetOutputPort())
    writer.SetFileName(str(path))
    writer.Write()
    if writer.GetErrorCode() or not path.is_file() or not path.stat().st_size:
        raise RuntimeError(f"Could not save Slicer view to {path.name}")
    return list(pixels.GetOutput().GetDimensions()[:2])


def capture(folder):
    folder = Path(folder)

    def read(name):
        return json.loads((folder / name).read_text(encoding="utf-8"))

    report = read("summary.json")
    start = read("initial_state.json")
    panel = slicer.ur12eSimulation
    panel.poll()
    live = panel.state
    if (panel.feedback_error or not live or live["source"] != "ros2_mock_hardware" or
            not live["ready"] or live["joint_names"] != start["joint_names"] or any(
                abs(a - b) >= 1e-5 for a, b in zip(live["joint_positions_rad"], start["joint_positions_rad"]))):
        raise RuntimeError("Slicer is not displaying the recorded mock start pose")

    def path_points(label):
        rows = read(report["paths"][label]["file"])["samples"]
        return [ras(row["tip_pose_base"]["position_m"]) for row in rows]

    def read_sphere(label):
        item, = report["scenes"][label]["objects"]
        primitive, = item["primitives"]
        return (primitive["pose_base"]["position_m"], primitive["dimensions_m"][0])

    baseline = path_points("baseline_clear")
    detour = path_points("replanned") if report["midpath_outcome"] == "detour_valid_at_samples" else []
    blocked = read(report["paths"]["baseline_with_sphere"]["file"])["samples"]
    collisions = [point for point, row in zip(baseline, blocked) if not row["valid"]]
    mid = report["midpoint_sphere"]
    goal = ras(report["target_pose_base"]["position_m"])
    bounds = np.asarray(baseline + detour + [goal])

    view = slicer.app.layoutManager().threeDWidget(0).threeDView()
    window = view.renderWindow()
    renderer = window.GetRenderers().GetFirstRenderer()
    camera = renderer.GetActiveCamera()
    previous_camera = vtk.vtkCamera()
    previous_camera.DeepCopy(camera)
    annotation = view.cornerAnnotation()
    annotation_visible = annotation.GetVisibility()
    controls = [panel.target_marker.GetDisplayNode(), panel.plan_path.GetDisplayNode()]
    controls += [node.GetDisplayNode() for node in slicer.util.getNodesByClass("vtkMRMLModelNode")
                 if node.GetTransformNodeID() == panel.target_transform.GetID()]
    visibility = [control.GetVisibility() for control in controls]
    results = {"slicer_version": slicer.app.applicationVersion, "captures": []}

    def render(name, title, detail, center, radius, color, obstruction_points=(), new_path=False):
        nodes = []
        actors = []
        try:
            nodes.append(model("Virtual keep-out sphere", sphere(ras(center), radius * 1000.0), RED, 0.27))
            nodes.append(model("Baseline MoveIt FK", polyline(baseline, 1.9), GOLD))
            if new_path:
                nodes.append(model("Replanned MoveIt FK", polyline(detour, 2.5), TEAL))
            for point in obstruction_points:
                nodes.append(model("Tip at a colliding robot sample", sphere(point, 3.0), RED))
            nodes.append(model("Start", sphere(baseline[0], 4.5), WHITE))
            nodes.append(model("Commanded needle-tip goal", sphere(goal, 5.0), color))
            actors = [text_actor(title, WHITE, .035, .96, 24), text_actor(detail, WHITE, .035, .90, 17),
                      text_actor("BASELINE   MoveIt FK", GOLD, .035, .17, 17),
                      text_actor("UR12e mock  /  Planning only", WHITE, .035, .055, 15)]
            if new_path:
                actors.append(text_actor("REPLAN   MoveIt FK", TEAL, .035, .125, 17))
            if obstruction_points:
                actors.append(text_actor("COLLIDING ROBOT SAMPLES", RED, .965, .055, 15))
                actors[-1].GetTextProperty().SetJustificationToRight()
            for actor in actors:
                renderer.AddActor2D(actor)
            renderer.ResetCameraClippingRange()
            path = folder / (name + ".png")
            size = screenshot(window, path)
            results["captures"].append({"file": path.name, "size_px": size, "sphere_center_m": center,
                                        "sphere_radius_m": radius})
        finally:
            for actor in actors:
                renderer.RemoveActor2D(actor)
            for node in nodes:
                display = node.GetDisplayNode()
                slicer.mrmlScene.RemoveNode(node)
                if display and display.GetScene():
                    slicer.mrmlScene.RemoveNode(display)

    try:
        for control in controls:
            control.SetVisibility(False)
        annotation.VisibilityOff()
        focal = (bounds.min(axis=0) + bounds.max(axis=0)) / 2.0 + np.array([0, 0, 45])
        camera.SetPosition(*(focal + [-900, -360, 300]))
        camera.SetFocalPoint(*focal)
        camera.SetViewUp(0, 0, 1)
        camera.ParallelProjectionOn()
        camera.SetParallelScale(max(170.0, float(np.ptp(bounds, axis=0).max()) * .68))
        results["camera"] = {"position_ras_mm": list(camera.GetPosition()), "focal_point_ras_mm": list(focal),
                             "view_up": list(camera.GetViewUp()), "parallel_scale_mm": camera.GetParallelScale()}
        render("01_display_only", "01  DISPLAY ONLY", "The sphere is absent from MoveIt's collision world",
               mid["center_m"], mid["radius_m"], GOLD)
        detail = (f"{len(collisions)} baseline samples collide  /  " +
                  ("Replan passes sampled checks" if detour else "No usable replan returned"))
        render("02_midpath_comparison", "02  SPHERE IN MOVEIT", detail,
               *read_sphere("midpath"), GOLD, obstruction_points=collisions, new_path=bool(detour))
        failure_code = report["cases"][-1]["moveit_error_code"]
        render("03_goal_blocked", "03  GOAL INSIDE SPHERE", f"Goal rejected  /  MoveIt code {failure_code}",
               *read_sphere("goal_blocked"), RED, obstruction_points=[baseline[-1]])
    finally:
        camera.DeepCopy(previous_camera)
        annotation.SetVisibility(annotation_visible)
        for control, visible in zip(controls, visibility):
            control.SetVisibility(visible)
        view.scheduleRender()
    return results
