#!/usr/bin/env python3
"""Fixed loopback GUI relay for this task's internal URSim Docker bridge.

No target/listen arguments are exposed. The lifecycle manager verifies the
container before launching this helper and owns its PID/process group.
"""
import os
import select
import signal
import socket
import socketserver
import time

import ursim_lifecycle as lifecycle


class GuiRelay(socketserver.BaseRequestHandler):
    def handle(self):
        try:
            with socket.create_connection((lifecycle.ROBOT_IP, 6080), timeout=5) as upstream:
                self.request.settimeout(30)
                upstream.settimeout(30)
                pair = (self.request, upstream)
                while True:
                    readable, _, _ = select.select(pair, [], [], 5)
                    for source in readable:
                        data = source.recv(65536)
                        if not data:
                            return
                        destination = upstream if source is self.request else self.request
                        destination.sendall(data)
        except (OSError, ValueError):
            # A disconnected browser or stopped container affects this session.
            return


class GuiServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    if os.geteuid() == 0:
        raise RuntimeError("Run the URSim GUI relay as the ordinary project user.")
    lifecycle.verified_container(lifecycle.Docker(), require_running=True)
    owner = None
    try:
        with GuiServer(("127.0.0.1", 6080), GuiRelay) as server:
            process = lifecycle.common().process(os.getpid())
            if not process or process["group"] != os.getpid():
                raise RuntimeError("The URSim GUI relay requires its own recorded process group.")
            owner = {"pid": os.getpid(), "group": os.getpgrp(), "start_ticks": process["start_ticks"],
                     "ready": True, "started_at": time.time(), "listen": "127.0.0.1:6080",
                     "target": lifecycle.ROBOT_IP + ":6080", "project_key": lifecycle.PROJECT_KEY}
            lifecycle.write_json(lifecycle.PROXY_OWNER, owner)

            def stopped(_number, _frame):
                raise SystemExit(0)

            signal.signal(signal.SIGTERM, stopped)
            signal.signal(signal.SIGINT, stopped)
            print("URSim GUI relay: 127.0.0.1:6080 -> " + lifecycle.ROBOT_IP + ":6080", flush=True)
            server.serve_forever(poll_interval=0.25)
    finally:
        if owner:
            owner.update(ready=False, stopped_at=time.time())
            lifecycle.write_json(lifecycle.PROXY_OWNER, owner)


if __name__ == "__main__":
    main()
