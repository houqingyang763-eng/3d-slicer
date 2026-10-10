"""Send one Python file to the classroom Slicer. Standard library only."""
from pathlib import Path
import argparse
import json
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def send(code, timeout=120):
    folder = ROOT / "work" / "bridge"
    state = json.loads((folder / "session.json").read_text(encoding="utf-8"))
    live_state = folder / f"state_{state['session']}.json"
    if live_state.exists():
        state = json.loads(live_state.read_text(encoding="utf-8"))
    if time.time() - state["heartbeat"] > 10:
        raise RuntimeError("Slicer is not responding. Open the classroom launcher first.")
    identifier = str(uuid.uuid4())
    inbox = folder / "inbox" / f"{identifier}.json"
    pending = inbox.with_suffix(".tmp")
    pending.write_text(json.dumps({"id": identifier, "session": state["session"], "code": code}), encoding="utf-8")
    pending.replace(inbox)
    outbox = folder / "outbox" / inbox.name
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if outbox.exists():
            result = json.loads(outbox.read_text(encoding="utf-8"))
            if not result.get("ok"):
                raise RuntimeError(result.get("error", "Unknown Slicer error"))
            return result
        time.sleep(0.2)
    raise TimeoutError(f"Slicer command timed out; response may still appear at {outbox}")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser()
    parser.add_argument("script", type=Path)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    print(json.dumps(send(args.script.read_text(encoding="utf-8"), args.timeout), ensure_ascii=False, indent=2))
