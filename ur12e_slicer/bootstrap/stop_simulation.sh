#!/usr/bin/env bash
# Ask the recorded launch process to stop, then verify its group has exited.
set -Eeuo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 - "$SCRIPT_DIR/../runtime/linux_launch.json" <<'PY'
import json, os, signal, sys, time
from pathlib import Path
p = Path(sys.argv[1])
if not p.exists():
    print("No simulation process was recorded; nothing to stop.")
    raise SystemExit(0)
state = json.loads(p.read_text(encoding="utf-8"))
pid = int(state["pid"])
proc = Path(f"/proc/{pid}")

def live_group_members():
    """Count all surviving group members, including children reparented to init."""
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = entry.joinpath("stat").read_text()
        except (FileNotFoundError, ProcessLookupError):
            continue
        fields = stat.rsplit(") ", 1)[1].split()
        if int(fields[2]) == pid and fields[0] not in ("Z", "X"):
            members.append({"pid": int(entry.name), "state": fields[0],
                            "name": stat.split("(", 1)[1].rsplit(") ", 1)[0]})
    return members


try:
    fields = proc.joinpath("stat").read_text().rsplit(") ", 1)[1].split()
    ticks = fields[19]
    if ticks != str(state["start_ticks"]) or int(fields[2]) != pid:
        raise SystemExit("Process identity did not match; no signal was sent.")
    if fields[0] not in ("Z", "X"):
        command = proc.joinpath("cmdline").read_bytes()
        if b"ur12e" not in command:
            raise SystemExit("Process identity did not match; no signal was sent.")
        # --noninteractive makes launch forward this once to each child.
        # Never signal the entire process group or send a second kill here.
        os.kill(pid, signal.SIGINT)
        print("Sent SIGINT to the recorded launch process; waiting for its group to exit.", flush=True)
except (FileNotFoundError, ProcessLookupError):
    # A launch process may exit before a child. Still inspect the old group.
    pass

deadline = time.monotonic() + 25.0
while True:
    members = live_group_members()
    if not members:
        print("Verified: the recorded simulation group has no live non-zombie processes.")
        raise SystemExit(0)
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise SystemExit("Simulation did not stop within 25 seconds; no extra kill was sent. "
                         "Remaining group members: " + json.dumps(members))
    time.sleep(min(0.2, remaining))
PY
