"""A local file queue, serviced on Slicer's GUI thread. No network listener."""
from pathlib import Path
import json
import os
import time
import traceback
import uuid

import qt
import slicer

ROOT = Path(__file__).resolve().parents[1]
QUEUE = ROOT / "work" / "bridge"
INBOX = QUEUE / "inbox"
OUTBOX = QUEUE / "outbox"


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


class LocalBridge:
    def __init__(self):
        INBOX.mkdir(parents=True, exist_ok=True)
        OUTBOX.mkdir(parents=True, exist_ok=True)
        self.session = str(uuid.uuid4())
        self.last_heartbeat = 0
        self.scope = {"slicer": slicer, "qt": qt, "ROOT": ROOT}
        self.state = {
            "session": self.session, "pid": os.getpid(), "heartbeat": time.time(),
            "slicer_version": slicer.app.applicationVersion,
            "revision": slicer.app.repositoryRevision,
        }
        write_json(QUEUE / "session.json", self.state)
        self.timer = qt.QTimer()
        self.timer.setInterval(250)
        self.timer.connect("timeout()", self.tick)
        self.timer.start()
        self.tick()

    def tick(self):
        if time.time() - self.last_heartbeat > 1:
            self.state["heartbeat"] = time.time()
            write_json(QUEUE / f"state_{self.session}.json", self.state)
            self.last_heartbeat = time.time()
        for path in sorted(INBOX.glob("*.json")):
            try:
                request = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if request.get("session") != self.session:
                continue
            response = {"id": request["id"], "session": self.session}
            try:
                self.scope["_result"] = None
                exec(compile(request["code"], f"<local-command-{request['id']}>", "exec"), self.scope)
                response.update(ok=True, result=self.scope.get("_result"))
                json.dumps(response)
            except Exception:
                response.update(ok=False, error=traceback.format_exc())
            write_json(OUTBOX / path.name, response)
            path.unlink()


def start():
    if not hasattr(slicer, "classroomBridge"):
        slicer.classroomBridge = LocalBridge()
    return slicer.classroomBridge
