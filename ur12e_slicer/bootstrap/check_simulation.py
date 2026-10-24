"""Read-only identity and readiness check for the recorded task launch."""
import json
import os
from pathlib import Path
import time

runtime = Path(__file__).resolve().parents[1] / "runtime"
result = {"running": False, "ready": False, "checked_at": time.time()}
try:
    record = json.loads((runtime / "linux_launch.json").read_text(encoding="utf-8-sig"))
    pid = int(record["pid"])
    proc = Path(f"/proc/{pid}")
    fields = proc.joinpath("stat").read_text().split(") ", 1)[1].split()
    command = proc.joinpath("cmdline").read_bytes()
    result.update({"pid": pid, "running": fields[19] == str(record["start_ticks"])
                   and fields[0] != "Z" and b"ur12e" in command and os.getpgid(pid) == pid})
    if result["running"]:
        state = json.loads((runtime / "state.json").read_text(encoding="utf-8"))
        age = time.time() - state["timestamp"]
        result.update({"feedback_age_s": age, "ready": state.get("source") == "ros2_mock_hardware"
                       and state.get("ready") is True and state.get("feedback_valid") is True
                       and 0 <= age <= 1 and state["timestamp"] >= record.get("started_at", 0),
                       "request_id": state.get("request_id"), "status": state.get("status")})
except (OSError, ValueError, KeyError) as error:
    result["detail"] = str(error)
print(json.dumps(result))
