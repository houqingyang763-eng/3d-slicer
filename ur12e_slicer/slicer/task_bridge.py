"""Local automation queue for this task's Slicer instance only.

The queue stays under this project's runtime directory. Other Slicer sessions
and the repository's classroom bridge are never selected or modified.
"""
from pathlib import Path
import json
import os
import time
import traceback
import uuid

import qt
import slicer

PROJECT = Path(__file__).resolve().parents[1]
QUEUE = PROJECT / "runtime" / "slicer_bridge"


def write_json(path, value):
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


class TaskBridge:
    def __init__(self):
        self.inbox = QUEUE / "inbox"
        self.outbox = QUEUE / "outbox"
        self.inbox.mkdir(parents=True, exist_ok=True)
        self.outbox.mkdir(parents=True, exist_ok=True)
        self.session = str(uuid.uuid4())
        self.last_heartbeat = 0
        self.scope = {"slicer": slicer, "qt": qt, "PROJECT": PROJECT}
        self.state = {"session": self.session, "pid": os.getpid(),
                      "heartbeat": time.time(), "slicer_version": slicer.app.applicationVersion,
                      "task": "ur12e_slicer", "queue": str(QUEUE)}
        write_json(QUEUE / "session.json", self.state)
        self.timer = qt.QTimer()
        self.timer.setInterval(250)
        self.timer.connect("timeout()", self.tick)
        self.timer.start()

    def tick(self):
        if time.time() - self.last_heartbeat > 1:
            self.state["heartbeat"] = time.time()
            write_json(QUEUE / ("state_" + self.session + ".json"), self.state)
            self.last_heartbeat = time.time()
        for path in sorted(self.inbox.glob("*.json")):
            try:
                request = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if request.get("session") != self.session:
                continue
            response = {"id": request["id"], "session": self.session}
            try:
                self.scope["_result"] = None
                exec(compile(request["code"], "<ur12e-command-" + request["id"] + ">", "exec"), self.scope)
                response.update(ok=True, result=self.scope.get("_result"))
                json.dumps(response)
            except Exception:
                response.update(ok=False, error=traceback.format_exc())
            write_json(self.outbox / path.name, response)
            path.unlink()


def start():
    if not hasattr(slicer, "ur12eTaskBridge"):
        slicer.ur12eTaskBridge = TaskBridge()
    return slicer.ur12eTaskBridge
