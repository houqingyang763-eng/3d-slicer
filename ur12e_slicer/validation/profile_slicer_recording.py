"""Profile real Qt recording and live feedback while leaving all targets untouched."""
import json
import time


class CaptureProfile:
    def __init__(self):
        self.panel = slicer.ur12eSimulation
        if self.panel.recording:
            raise RuntimeError("Another recording is active")
        self.folder = PROJECT / "output" / "capture_benchmark"
        self.folder.mkdir(parents=True, exist_ok=True)
        self.observations = []
        self.started = time.monotonic()
        self.panel.toggle_recording()
        self.recording = str(self.panel.recording)
        self.timer = qt.QTimer()
        self.timer.setInterval(100)
        self.timer.connect("timeout()", self.tick)
        self.timer.start()

    def tick(self):
        elapsed = time.monotonic() - self.started
        self.observations.append({"elapsed_s": elapsed,
                                  "feedback_timestamp": (self.panel.state or {}).get("timestamp")})
        if elapsed < 4.5:
            return
        self.timer.stop()
        self.panel.toggle_recording()
        frames = self.panel.recorded_frames
        updates = len({sample["feedback_timestamp"] for sample in self.observations})
        report = {"duration_s": elapsed, "frames": len(frames), "feedback_updates": updates,
                  "maximum_capture_s": max((frame["capture_duration_s"] for frame in frames), default=0),
                  "recording_directory": self.recording,
                  "passed": len(frames) >= 20 and updates >= 10,
                  "observations": self.observations}
        (self.folder / "timer_profile.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


slicer.ur12eCaptureProfile = CaptureProfile()
_result = {"started": True, "target_sent": False}
