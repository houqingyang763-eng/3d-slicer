"""Measure local Slicer capture cost without changing or sending robot targets."""
import time
import qt

folder = PROJECT / "output" / "capture_benchmark"
folder.mkdir(parents=True, exist_ok=True)
measurements = []
for index in range(2):
    started = time.perf_counter()
    pixmap = slicer.util.mainWindow().grab()
    grabbed = time.perf_counter()
    reduced = pixmap.scaledToWidth(1600, qt.Qt.SmoothTransformation)
    scaled = time.perf_counter()
    saved = reduced.save(str(folder / ("capture_%d.png" % index)), "PNG", 100)
    completed = time.perf_counter()
    measurements.append({"grab_s": grabbed-started, "scale_s": scaled-grabbed,
                         "save_s": completed-scaled, "total_s": completed-started,
                         "saved": bool(saved), "width": reduced.width(), "height": reduced.height()})
_result = measurements
