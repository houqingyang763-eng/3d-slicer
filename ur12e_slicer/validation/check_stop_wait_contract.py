"""Isolated stop safety checks: fake /proc, clock and signals; no ROS action."""
import contextlib
import io
import json
import math
import os
from pathlib import Path
import pathlib
import signal
import sys
import time
from unittest.mock import patch

root = Path(__file__).resolve().parents[1]
source = (root / "bootstrap/stop_simulation.sh").read_text(encoding="utf-8")
code = source.split("<<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
output_path = root / "output/stop_wait_contract_checks.json"
cases = [
    {"name": "no_record", "record": False, "leader": False, "child": False, "ok": True, "signals": 0},
    {"name": "already_stopped", "leader": False, "child": False, "ok": True, "signals": 0},
    {"name": "identity_mismatch_refused", "mismatch": True, "ok": False, "signals": 0},
    {"name": "delayed_child_waits_15_seconds", "child_exit": 15.0, "ok": True, "signals": 1, "min_wait": 15.0},
    {"name": "persistent_child_fails_at_25_seconds", "child_exit": math.inf, "ok": False, "signals": 1, "min_wait": 25.0},
    {"name": "zombie_child_does_not_block", "child_exit": math.inf, "child_state": "Z", "leader_exit": 0.0, "ok": True, "signals": 1},
    {"name": "orphan_child_still_waited_without_resignal", "leader": False, "child_exit": 0.6, "ok": True, "signals": 0, "min_wait": 0.6},
]
reports = []
for case in cases:
    clock = [0.0]
    sent = [False]

    def alive(pid):
        if pid == 12345:
            return case.get("leader", True) and (not sent[0] or clock[0] < case.get("leader_exit", 0.1))
        if pid == 12346:
            return case.get("child", True) and clock[0] < case.get("child_exit", 0.2)
        return pid == 777  # An unrelated group must never block or be signalled.

    class FakePath:
        def __init__(self, value):
            self.value = str(value)

        @property
        def name(self):
            return self.value.rsplit("/", 1)[-1]

        def exists(self):
            assert self.value == "/fake/linux_launch.json"
            return case.get("record", True)

        def joinpath(self, name):
            return FakePath(self.value + "/" + name)

        def iterdir(self):
            assert self.value == "/proc"
            return [FakePath("/proc/" + str(pid)) for pid in (12345, 12346, 777) if alive(pid)]

        def read_text(self, **kwargs):
            if self.value == "/fake/linux_launch.json":
                return json.dumps({"pid": 12345, "start_ticks": "123"})
            pid = int(self.value.split("/")[2])
            if not alive(pid):
                raise FileNotFoundError(self.value)
            state = case.get("child_state", "S") if pid == 12346 else "S"
            group = 12345 if pid != 777 else 777
            ticks = "wrong" if case.get("mismatch") and pid == 12345 else "123"
            return "{} (test) ".format(pid) + " ".join([state, "1", str(group)] + ["0"] * 16 + [ticks])

        def read_bytes(self):
            return b"ros2 launch --noninteractive ur12e_slicer_bridge mock.launch.py"

    def send(pid, number):
        assert pid == 12345 and number == signal.SIGINT
        sent[0] = True

    def advance(seconds):
        clock[0] += seconds

    output = io.StringIO()
    with patch.object(sys, "argv", ["stop", "/fake/linux_launch.json"]), \
            patch.object(pathlib, "Path", FakePath), \
            patch.object(os, "kill", side_effect=send) as kill, \
            patch.object(os, "killpg", create=True) as killpg, \
            patch.object(time, "monotonic", side_effect=lambda: clock[0]), \
            patch.object(time, "sleep", side_effect=advance), contextlib.redirect_stdout(output):
        try:
            exec(compile(code, "stop_simulation_inline", "exec"), {})
            result = 0
        except SystemExit as exc:
            result = exc.code
        assert (result == 0) == case["ok"], (case["name"], result)
        assert kill.call_count == case["signals"], (case["name"], kill.call_count)
        killpg.assert_not_called()
    assert case.get("min_wait", 0) <= clock[0] <= 25.0, (case["name"], clock[0])
    if not case["ok"] and case["name"].startswith("persistent"):
        assert '"pid": 12346' in result and "no extra kill" in result
    reports.append({"name": case["name"], "passed": True, "simulated_elapsed_s": clock[0],
                    "signal_count": case["signals"], "exit_result": result,
                    "stdout": output.getvalue().strip()})
report = {"source": "fake process table, fake monotonic clock, mocked signals; no node started or stopped",
          "cases": reports, "passed": True}
output_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print(json.dumps(report, indent=2))
