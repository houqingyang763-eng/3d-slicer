"""Execute a Python file in the dedicated UR12e Slicer GUI thread."""
from pathlib import Path
import argparse
import json
import os
import sys
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]


def send(code, timeout=30):
    folder = PROJECT / "runtime" / "slicer_bridge"
    state = json.loads((folder / "session.json").read_text(encoding="utf-8"))
    state = json.loads((folder / ("state_" + state["session"] + ".json")).read_text(encoding="utf-8"))
    if state.get("task") != "ur12e_slicer" or time.time() - state["heartbeat"] > 10:
        raise RuntimeError("Dedicated UR12e Slicer is not responding. Use its launcher first.")
    identifier = str(uuid.uuid4())
    inbox = folder / "inbox" / (identifier + ".json")
    temporary = inbox.with_suffix(".tmp")
    temporary.write_text(json.dumps({"id": identifier, "session": state["session"], "code": code}), encoding="utf-8")
    os.replace(temporary, inbox)
    outbox = folder / "outbox" / inbox.name
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if outbox.exists():
            result = json.loads(outbox.read_text(encoding="utf-8"))
            if not result.get("ok"):
                raise RuntimeError(result.get("error", "Unknown Slicer error"))
            return result
        time.sleep(0.2)
    raise TimeoutError("Slicer response did not arrive: " + str(outbox))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser()
    parser.add_argument("script", type=Path)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    print(json.dumps(send(args.script.read_text(encoding="utf-8"), args.timeout), ensure_ascii=False, indent=2))
