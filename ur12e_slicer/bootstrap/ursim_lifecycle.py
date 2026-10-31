#!/usr/bin/env python3
"""Manage this project's local UR12e URSim; no physical-robot endpoint exists.

Docker images, the External Control URCap and its program must already be
prepared. Status is read-only. Startup and stop act only on verified project
containers/processes and preserve logs and mounted program files.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import stat
import subprocess
import sys
import time
import traceback


PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent
RUNTIME = PROJECT / "runtime"
PROJECT_KEY = hashlib.sha256(str(REPO.resolve()).encode()).hexdigest()[:12]
NAME = "ur12e-slicer-ursim-" + PROJECT_KEY
IMAGE = "universalrobots/ursim_e-series@sha256:f1211a7feb4252135b1439f603f4e17ff826b4b71574f3963c868efe0498b4a9"
SUBNET, GATEWAY, ROBOT_IP = "192.168.62.0/24", "192.168.62.1", "192.168.62.2"
LABELS = {"ur12e_slicer.task": "ursim", "ur12e_slicer.project_key": PROJECT_KEY}
BACKEND_OWNER = RUNTIME / "ursim_backend.json"
CONTAINER_OWNER = RUNTIME / "ursim_container.json"
PROXY_OWNER = RUNTIME / "ursim_gui_proxy.json"


def common():
    import linux_lifecycle
    return linux_lifecycle


def cache_root():
    base = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))).expanduser()
    return base / "ur12e_slicer" / PROJECT_KEY / "ursim"


def control_runtime():
    return (cache_root() / "bridge").resolve()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    common().write_json(path, value)


def stamp():
    return common().stamp()


class Docker:
    """Resolve one local Unix socket, then pin every subsequent Docker call."""
    def __init__(self):
        env = os.environ.copy()
        explicit_host = env.get("DOCKER_HOST") if not env.get("DOCKER_CONTEXT") else None
        if explicit_host:
            endpoint = explicit_host
        else:
            result = subprocess.run(["docker", "context", "inspect"], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError("Cannot inspect local Docker context: " + result.stderr.strip())
            contexts = json.loads(result.stdout)
            endpoint = contexts[0]["Endpoints"]["docker"]["Host"]
        if not isinstance(endpoint, str) or not endpoint.startswith("unix://"):
            raise RuntimeError("URSim requires a local Unix Docker socket; remote contexts are refused.")
        sock = Path(endpoint[len("unix://"):])
        if not sock.is_absolute() or not sock.exists() or not stat.S_ISSOCK(sock.stat().st_mode):
            raise RuntimeError("Docker endpoint is not an existing local Unix socket: " + str(sock))
        self.endpoint = endpoint
        for name in ("DOCKER_HOST", "DOCKER_CONTEXT", "DOCKER_TLS_VERIFY", "DOCKER_CERT_PATH"):
            env.pop(name, None)
        self.env = env

    def call(self, *arguments, allow_missing=False):
        result = subprocess.run(["docker", "--host", self.endpoint, *arguments],
                                env=self.env, capture_output=True, text=True)
        if result.returncode:
            message = (result.stderr or result.stdout).strip()
            if allow_missing and any(word in message.lower() for word in ("no such", "not found")):
                return None
            raise RuntimeError("Docker " + arguments[0] + " failed: " + message)
        return result.stdout

    def inspect(self, kind, name):
        output = self.call(kind, "inspect", name, allow_missing=True)
        return json.loads(output)[0] if output is not None else None


def validate_network(network):
    if not network or network.get("Name") != NAME:
        raise RuntimeError("The recorded URSim Docker network is absent or misnamed.")
    if network.get("Driver") != "bridge" or network.get("Internal") is not True:
        raise RuntimeError("URSim requires its own internal Docker bridge.")
    if any(network.get("Labels", {}).get(key) != value for key, value in LABELS.items()):
        raise RuntimeError("URSim network ownership labels do not match this project.")
    configs = network.get("IPAM", {}).get("Config", [])
    if len(configs) != 1 or configs[0].get("Subnet") != SUBNET or configs[0].get("Gateway") != GATEWAY:
        raise RuntimeError("URSim network subnet/gateway does not match the fixed local endpoint.")
    if any(item.get("Name") != NAME for item in network.get("Containers", {}).values()):
        raise RuntimeError("Another container is attached to this project's URSim network.")
    return network


def validate_container(container, network, image):
    validate_network(network)
    if not container or container.get("Name", "").lstrip("/") != NAME:
        raise RuntimeError("The URSim container is absent or misnamed.")
    config = container.get("Config", {})
    if any(config.get("Labels", {}).get(key) != value for key, value in LABELS.items()):
        raise RuntimeError("URSim container ownership labels do not match this project.")
    if not image or IMAGE not in image.get("RepoDigests", []):
        raise RuntimeError("The pinned URSim image is not installed with its expected digest.")
    if config.get("Image") != IMAGE or container.get("Image") != image.get("Id"):
        raise RuntimeError("URSim container image does not match the pinned local image.")
    robot_models = [entry for entry in config.get("Env", []) if entry.startswith("ROBOT_MODEL=")]
    if robot_models != ["ROBOT_MODEL=UR12"]:
        raise RuntimeError("URSim container must explicitly select ROBOT_MODEL=UR12.")
    host = container.get("HostConfig", {})
    if host.get("Privileged") or host.get("NetworkMode") != NAME:
        raise RuntimeError("Unexpected URSim privileges or network mode.")
    if host.get("PortBindings") != {"6080/tcp": [{"HostIp": "127.0.0.1", "HostPort": "6080"}]}:
        raise RuntimeError("Only the loopback-bound URSim GUI port may be published.")
    networks = container.get("NetworkSettings", {}).get("Networks", {})
    if set(networks) != {NAME}:
        raise RuntimeError("URSim container is attached to an unexpected network.")
    attached = networks[NAME]
    if attached.get("NetworkID") != network.get("Id") or attached.get("IPAMConfig", {}).get("IPv4Address") != ROBOT_IP:
        raise RuntimeError("URSim fixed IP or network identity does not match.")
    if container.get("State", {}).get("Running") and attached.get("IPAddress") != ROBOT_IP:
        raise RuntimeError("The running URSim container does not own the fixed local IP.")
    if any(identifier != container.get("Id") for identifier in network.get("Containers", {})):
        raise RuntimeError("The URSim network has an unexpected container endpoint.")
    mounted = {entry.get("Destination"): entry for entry in container.get("Mounts", [])}
    for destination, folder in (("/ursim/programs", "programs"), ("/urcaps", "urcaps")):
        entry = mounted.get(destination, {})
        if entry.get("Type") != "bind" or Path(entry.get("Source", "/")).resolve() != (cache_root() / folder).resolve():
            raise RuntimeError("URSim program/URCap mount does not belong to this project.")
    return container


def verified_container(docker, require_running=False):
    network = docker.inspect("network", NAME)
    image = docker.inspect("image", IMAGE)
    container = validate_container(docker.inspect("container", NAME), network, image)
    if require_running and not container.get("State", {}).get("Running"):
        raise RuntimeError("The verified local URSim container is not running.")
    return container


def prepare_container(docker):
    programs, urcaps = cache_root() / "programs", cache_root() / "urcaps"
    if not (programs / "external_control.urp").is_file() or not urcaps.is_dir() or not list(urcaps.glob("*.jar")):
        raise RuntimeError("Prepare this project's External Control URCap and external_control.urp first; no downloads are performed.")
    image = docker.inspect("image", IMAGE)
    if not image or IMAGE not in image.get("RepoDigests", []):
        raise RuntimeError("Pinned URSim image is not installed; no image pull is performed by this manager.")
    network = docker.inspect("network", NAME)
    if network is None:
        labels = [part for key, value in LABELS.items() for part in ("--label", key + "=" + value)]
        docker.call("network", "create", "--driver", "bridge", "--internal", "--subnet", SUBNET,
                    "--gateway", GATEWAY, *labels, NAME)
        network = docker.inspect("network", NAME)
    validate_network(network)
    if docker.inspect("container", NAME) is None:
        labels = [part for key, value in LABELS.items() for part in ("--label", key + "=" + value)]
        docker.call("container", "create", "--name", NAME, "--network", NAME, "--ip", ROBOT_IP,
                    "--env", "ROBOT_MODEL=UR12", "--publish", "127.0.0.1:6080:6080", *labels,
                    "--mount", "type=bind,src=" + str(programs) + ",dst=/ursim/programs",
                    "--mount", "type=bind,src=" + str(urcaps) + ",dst=/urcaps,readonly", IMAGE)
    container = verified_container(docker)
    write_json(CONTAINER_OWNER, {"container_id": container["Id"], "network_id": network["Id"],
               "name": NAME, "project_key": PROJECT_KEY, "image": IMAGE,
               "docker_endpoint": docker.endpoint, "recorded_at": time.time()})
    created_running = not container["State"]["Running"]
    if created_running:
        docker.call("container", "start", NAME)
    return verified_container(docker, require_running=True), created_running


def mock_is_live():
    owner_path = RUNTIME / "linux_launch.json"
    if not owner_path.exists():
        return False
    owner = read_json(owner_path)
    process = common().process(owner["pid"])
    return bool((process and process["state"] not in ("Z", "X")) or common().group_members(int(owner["pid"])))


def backend_status():
    result = {"running": False, "ready": False, "source": "ros2_ursim"}
    if not BACKEND_OWNER.exists():
        return result
    try:
        owner = read_json(BACKEND_OWNER)
        process = common().process(owner["pid"])
        members = common().group_members(int(owner["group"]))
        result.update(pid=owner["pid"], group=owner["group"], running=bool(members))
        identity = bool(process and process["state"] not in ("Z", "X")
                        and str(process["start_ticks"]) == str(owner["start_ticks"])
                        and process["group"] == owner["group"] == owner["pid"]
                        and b"ursim.launch.py" in process["command"]
                        and b"ur12e_slicer_bridge" in process["command"]
                        and str(control_runtime()).encode() in process["command"])
        result["identity_valid"] = identity
        if not identity:
            if members:
                result["error"] = "URSim backend process identity does not match; no process may be signalled."
            return result
        state = read_json(control_runtime() / "state.json")
        age = time.time() - state["timestamp"]
        connected = bool(state.get("source") == "ros2_ursim" and state.get("feedback_valid") is True
                         and 0 <= age <= 1 and state["timestamp"] >= owner["started_at"])
        result.update(feedback_age_s=age, status=state.get("status"), feedback_valid=state.get("feedback_valid"),
                      feedback_connected=connected, ready=bool(connected and state.get("ready") is True))
    except (OSError, ValueError, KeyError, TypeError) as error:
        result["error"] = str(error)
    return result


def status():
    result = {"backend": backend_status(), "slicer": common().slicer_status(),
              "gui_proxy": proxy_status(),
              "control_runtime": str(control_runtime()),
              "simulation_only": True, "source": "ros2_ursim", "ready": False,
              "container": {"name": NAME, "running": False, "verified": False}}
    try:
        docker = Docker()
        if docker.inspect("container", NAME) is not None:
            container = verified_container(docker)
            result["container"].update(running=container["State"]["Running"], verified=True,
                                       id=container["Id"], image=IMAGE, ip=ROBOT_IP, network=NAME)
    except Exception as error:
        result["container"]["error"] = str(error)
    result["ready"] = bool(result["backend"]["ready"] and result["container"]["verified"] and result["container"]["running"])
    return result


def proxy_status():
    result = {"running": False, "ready": False, "listen": "127.0.0.1:6080", "target": ROBOT_IP + ":6080"}
    if not PROXY_OWNER.exists():
        return result
    try:
        owner = read_json(PROXY_OWNER)
        process = common().process(owner["pid"])
        members = common().group_members(int(owner["group"]))
        identity = bool(process and process["state"] not in ("Z", "X")
                        and str(process["start_ticks"]) == str(owner["start_ticks"])
                        and process["group"] == owner["group"] == owner["pid"]
                        and str(PROJECT / "bootstrap/ursim_gui_proxy.py").encode() in process["command"])
        result.update(pid=owner["pid"], group=owner["group"], running=bool(members), identity_valid=identity,
                      ready=bool(identity and owner.get("ready") is True and owner.get("target") == ROBOT_IP + ":6080"
                                 and owner.get("listen") == "127.0.0.1:6080"))
        if members and not identity:
            result["error"] = "GUI proxy process identity does not match; no signal may be sent."
    except (OSError, ValueError, KeyError, TypeError) as error:
        result["error"] = str(error)
    return result


def start_proxy(logs):
    before = proxy_status()
    if before["running"]:
        if not before["ready"]:
            raise RuntimeError(before.get("error", "Owned URSim GUI proxy is not ready."))
        return before, False
    path = logs / ("gui_proxy_" + stamp() + ".log")
    with path.open("wb") as stream:
        child = subprocess.Popen([sys.executable, str(PROJECT / "bootstrap/ursim_gui_proxy.py")],
                                 cwd=REPO, stdout=stream, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        state = proxy_status()
        if state["ready"] and state["pid"] == child.pid:
            return state, True
        if child.poll() is not None:
            raise RuntimeError("URSim loopback GUI proxy exited (code %s); see %s" % (child.returncode, path))
        time.sleep(0.1)
    raise RuntimeError("URSim loopback GUI proxy did not become ready; inspect " + str(path))


def stop_proxy():
    if not PROXY_OWNER.exists():
        return
    owner = read_json(PROXY_OWNER)
    state = proxy_status()
    if state["running"]:
        if not state.get("identity_valid"):
            raise RuntimeError(state.get("error", "URSim GUI proxy identity mismatch; no signal sent."))
        os.kill(owner["pid"], signal.SIGTERM)
    wait_for(lambda: not common().group_members(int(owner["group"])), 15,
             "URSim GUI proxy group remains active; no forced kill sent")
    owner.update(ready=False, stopped_at=time.time())
    write_json(PROXY_OWNER, owner)


def dashboard(docker, command):
    if "\n" in command or "\r" in command:
        raise ValueError("Dashboard commands must be one line.")
    verified_container(docker, require_running=True)
    with socket.create_connection((ROBOT_IP, 29999), timeout=5) as connection:
        with connection.makefile("rwb", buffering=0) as stream:
            banner = stream.readline().decode("utf-8", errors="replace").strip()
            if "Universal Robots Dashboard Server" not in banner:
                raise RuntimeError("Unexpected dashboard banner at verified URSim endpoint.")
            stream.write((command + "\n").encode())
            response = stream.readline().decode("utf-8", errors="replace").strip()
    log = RUNTIME / "ursim_logs/dashboard.jsonl"
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"timestamp": time.time(), "container": NAME,
                                "command": command, "response": response}) + "\n")
    return response


def wait_for(check, timeout, message):
    deadline, last_error = time.monotonic() + timeout, None
    while time.monotonic() < deadline:
        try:
            value = check()
            if value:
                return value
        except (OSError, RuntimeError) as error:
            last_error = str(error)
        time.sleep(0.5)
    raise RuntimeError(message + (": " + last_error if last_error else ""))


def workspace_path():
    return Path(os.environ.get("UR12E_BUILD_DIR", str(cache_root().parent / "ros2_ws"))).expanduser().resolve()


def build_workspace(logs):
    workspace = workspace_path()
    workspace.mkdir(parents=True, exist_ok=True)
    script = """set -e
source /opt/ros/jazzy/setup.bash
cd -- "$1"
colcon --log-base "$1/log" build --base-paths "$2" --build-base "$1/build" --install-base "$1/install" --packages-select ur12e_slicer_bridge --symlink-install
"""
    path = logs / ("build_" + stamp() + ".log")
    with path.open("wb") as stream:
        completed = subprocess.run(["bash", "-c", script, "ur12e-ursim-build", str(workspace), str(PROJECT / "ros2_ws/src")],
                                   cwd=REPO, stdout=stream, stderr=subprocess.STDOUT)
    if completed.returncode:
        raise RuntimeError("URSim bridge build failed (exit %d); see %s" % (completed.returncode, path))
    return workspace


def launch_backend(workspace, logs):
    control_runtime().mkdir(parents=True, exist_ok=True)
    script = """set -e
source /opt/ros/jazzy/setup.bash
source "$1/install/setup.bash"
exec ros2 launch --noninteractive ur12e_slicer_bridge ursim.launch.py "runtime_dir:=$2" launch_rviz:=false
"""
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID="62", ROS_LOCALHOST_ONLY="1")
    log_path = logs / ("backend_" + stamp() + ".log")
    with log_path.open("wb") as stream:
        child = subprocess.Popen(["bash", "-c", script, "ur12e-ursim-backend", str(workspace), str(control_runtime())],
                                 cwd=REPO, env=environment, stdout=stream, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
    process = common().process(child.pid)
    if not process:
        raise RuntimeError("URSim backend exited immediately; see " + str(log_path))
    write_json(BACKEND_OWNER, {"pid": child.pid, "group": child.pid, "start_ticks": process["start_ticks"],
               "started_at": time.time(), "source": "ros2_ursim", "log": str(log_path),
               "domain_id": 62, "robot_ip": ROBOT_IP, "container": NAME, "workspace": str(workspace),
               "control_runtime": str(control_runtime())})


def start_frontend(executable, logs):
    frontend = common().slicer_status()
    if frontend["running"]:
        if not frontend["ready"]:
            raise RuntimeError("Owned Slicer is running without a fresh panel; stop it before restarting.")
        session = read_json(RUNTIME / "slicer_session.json")
        if Path(session.get("control_runtime", str(RUNTIME))).resolve() != control_runtime():
            raise RuntimeError("Owned Slicer uses another control directory; stop it before starting URSim.")
        return frontend, False
    environment = os.environ.copy()
    environment["UR12E_CONTROL_RUNTIME_DIR"] = str(control_runtime())
    for key in ("PYTHONPATH", "PYTHONHOME", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH", "LD_LIBRARY_PATH"):
        environment.pop(key, None)
    with (logs / ("slicer_" + stamp() + ".log")).open("wb") as stream:
        child = subprocess.Popen([str(executable), "--no-splash", "--ignore-slicerrc", "--python-script",
                                  str(PROJECT / "slicer/start_slicer.py")], cwd=REPO, env=environment,
                                 stdout=stream, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                 start_new_session=True, close_fds=True)
    process = common().process(child.pid)
    if not process:
        raise RuntimeError("Slicer exited immediately; inspect URSim logs.")
    write_json(RUNTIME / "linux_slicer.json", {"pid": child.pid, "group": child.pid,
               "start_ticks": process["start_ticks"], "executable": str(executable)})
    try:
        return common().wait_ready(common().slicer_status, 120, "Slicer URSim panel"), True
    except BaseException:
        stop_frontend()
        raise


def stop_backend():
    if not BACKEND_OWNER.exists():
        return
    owner = read_json(BACKEND_OWNER)
    process = common().process(owner["pid"])
    if process and process["state"] not in ("Z", "X"):
        valid = (str(process["start_ticks"]) == str(owner["start_ticks"])
                 and process["group"] == owner["group"] == owner["pid"]
                 and b"ursim.launch.py" in process["command"]
                 and b"ur12e_slicer_bridge" in process["command"]
                 and str(control_runtime()).encode() in process["command"])
        if not valid:
            raise RuntimeError("URSim backend PID/start ticks/command/group mismatch; no signal sent.")
        os.kill(owner["pid"], signal.SIGINT)
    wait_for(lambda: not common().group_members(int(owner["group"])), 30,
             "URSim backend group remains active; logs and owner record retained, no forced kill sent")
    owner["stopped_at"] = time.time()
    write_json(BACKEND_OWNER, owner)


def stop_frontend():
    owner_path = RUNTIME / "linux_slicer.json"
    if owner_path.exists():
        owner = read_json(owner_path)
        process = common().process(owner["pid"])
        if process and process["state"] not in ("Z", "X"):
            if (str(process["start_ticks"]) != str(owner["start_ticks"])
                    or process["group"] != owner["group"] or owner["group"] != owner["pid"]
                    or str(PROJECT / "slicer/start_slicer.py").encode() not in process["command"]):
                raise RuntimeError("Slicer PID/start ticks/command/group mismatch; no signal sent.")
    common().stop_slicer()


def stop_container(docker):
    if not CONTAINER_OWNER.exists():
        return
    owner = read_json(CONTAINER_OWNER)
    container = verified_container(docker)
    network = docker.inspect("network", NAME)
    if (container["Id"] != owner["container_id"] or network["Id"] != owner["network_id"]
            or owner.get("docker_endpoint") != docker.endpoint or owner.get("project_key") != PROJECT_KEY):
        raise RuntimeError("Recorded URSim container/network/socket identity changed; container was not stopped.")
    if container["State"]["Running"]:
        docker.call("container", "stop", "--time", "30", container["Id"])
    if verified_container(docker)["State"]["Running"]:
        raise RuntimeError("URSim container is still running after stop.")
    owner["stopped_at"] = time.time()
    write_json(CONTAINER_OWNER, owner)


def stop_all(backend_only=False):
    if mock_is_live():
        raise RuntimeError("The mock backend is active; this entrypoint cannot stop it or its frontend.")
    errors = []
    if not backend_only:
        try:
            stop_frontend()
        except Exception as error:
            errors.append(str(error))
    try:
        stop_backend()
    except Exception as error:
        errors.append(str(error))
    if errors:
        raise RuntimeError("; ".join(errors))
    stop_proxy()
    if CONTAINER_OWNER.exists():
        stop_container(Docker())
    print("Stopped recorded URSim task processes and container; program files, owner records and logs retained.", flush=True)


def start_all(backend_only=False, simulator_only=False):
    if mock_is_live():
        raise RuntimeError("This project's mock backend is active; stop it with its own entrypoint first.")
    executable = None if backend_only or simulator_only else common().slicer_executable()
    before = backend_status()
    if before["running"] and not before.get("identity_valid"):
        raise RuntimeError(before.get("error", "URSim backend identity mismatch."))
    logs = RUNTIME / "ursim_logs"
    logs.mkdir(parents=True, exist_ok=True)
    docker = Docker()
    container_started = backend_created = frontend_created = proxy_created = False
    try:
        _, container_started = prepare_container(docker)
        wait_for(lambda: "Robotmode:" in dashboard(docker, "robotmode"), 120, "URSim dashboard did not become ready")
        proxy, proxy_created = start_proxy(logs)
        if simulator_only:
            result = {"simulator_only": True, "simulation_only": True, "container": NAME, "network": NAME,
                      "gui_proxy": proxy, "gui_url": "http://127.0.0.1:6080/vnc.html",
                      "backend": before, "source": "ros2_ursim"}
            print(json.dumps(result, indent=2), flush=True)
            return result
        if not before["running"]:
            workspace = build_workspace(logs)
            verified_container(docker, require_running=True)
            launch_backend(workspace, logs)
            backend_created = True
        mode = dashboard(docker, "robotmode")
        if not any(value in mode for value in ("IDLE", "RUNNING")):
            response = dashboard(docker, "power on")
            if "failed" in response.lower():
                raise RuntimeError("URSim power-on failed: " + response)
            wait_for(lambda: any(value in dashboard(docker, "robotmode") for value in ("IDLE", "RUNNING")), 60, "URSim did not power on")
        if "RUNNING" not in dashboard(docker, "robotmode"):
            response = dashboard(docker, "brake release")
            if "failed" in response.lower():
                raise RuntimeError("URSim brake release failed: " + response)
            wait_for(lambda: "RUNNING" in dashboard(docker, "robotmode"), 60, "URSim brakes did not release")
        # RTDE joint/TF feedback is available before External Control starts.
        # Wait for that connection so Play does not race a still-starting driver.
        wait_for(lambda: backend_status().get("feedback_connected"), 120,
                 "The URSim ROS driver did not establish fresh joint/TF feedback")
        playing = "PLAYING" in dashboard(docker, "programState")
        if playing and "/ursim/programs/external_control.urp" not in dashboard(docker, "get loaded program"):
            raise RuntimeError("The task URSim container is running an unexpected program; inspect it before restarting.")
        if not playing:
            response = dashboard(docker, "load /ursim/programs/external_control.urp")
            if not response.lower().startswith("loading program:"):
                raise RuntimeError("URSim External Control program load failed: " + response)
            response = dashboard(docker, "play")
            if "starting program" not in response.lower():
                raise RuntimeError("URSim External Control play failed: " + response)
        wait_for(lambda: "PLAYING" in dashboard(docker, "programState"), 30, "URSim External Control program did not start")
        backend = common().wait_ready(backend_status, 120, "URSim ROS/MoveIt backend")
        verified_container(docker, require_running=True)
        frontend = common().slicer_status()
        if not backend_only:
            frontend, frontend_created = start_frontend(executable, logs)
        result = {"backend": backend, "slicer": frontend, "simulation_only": True, "source": "ros2_ursim",
                  "container": NAME, "network": NAME, "ip": ROBOT_IP, "gui_proxy": proxy,
                  "control_runtime": str(control_runtime()),
                  "gui_url": "http://127.0.0.1:6080/vnc.html"}
        print(json.dumps(result, indent=2), flush=True)
        return result
    except BaseException:
        failure = {"failed_at": time.time(), "error": traceback.format_exc(), "cleanup_errors": []}
        if frontend_created:
            try:
                stop_frontend()
            except Exception as error:
                failure["cleanup_errors"].append("Slicer: " + str(error))
        if backend_created:
            try:
                stop_backend()
            except Exception as error:
                failure["cleanup_errors"].append("Backend: " + str(error))
        if proxy_created:
            try:
                stop_proxy()
            except Exception as error:
                failure["cleanup_errors"].append("GUI proxy: " + str(error))
        if container_started:
            if failure["cleanup_errors"]:
                failure["cleanup_errors"].append("Container left running because process cleanup was not confirmed.")
            else:
                try:
                    stop_container(docker)
                except Exception as error:
                    failure["cleanup_errors"].append("Container: " + str(error))
        write_json(logs / ("start_failure_" + stamp() + ".json"), failure)
        raise


def check_slicer_acceptance(folder, lifecycle_result, timeout=180, sender=None):
    """Require a fresh, completed Slicer target/plan/execute/feedback run."""
    if sender is None:
        sys.path.insert(0, str(PROJECT / "slicer"))
        from task_client import send
        sender = send
    status_path = PROJECT / "output/ursim_slicer_acceptance_status.json"
    previous_folder = None
    if status_path.exists():
        previous_folder = read_json(status_path).get("output_directory")
    started_at = time.time()
    deadline = time.monotonic() + timeout
    acceptance_folder = None
    passed = False
    try:
        source = (PROJECT / "validation/start_ursim_slicer_acceptance.py").read_text(encoding="utf-8")
        response = sender(source, timeout=30)
        write_json(folder / "slicer_start.json", response)
        if response.get("ok") is not True or not isinstance(response.get("result"), dict):
            raise RuntimeError("Slicer did not acknowledge the URSim acceptance start.")
        location = response["result"].get("output_directory")
        if not isinstance(location, str) or not location or location == previous_folder:
            raise RuntimeError("Slicer acceptance did not return a new output directory.")
        acceptance_folder = Path(location).resolve()
        base = (PROJECT / "output/ursim_slicer_acceptance").resolve()
        relative = acceptance_folder.relative_to(base)
        if len(relative.parts) != 1 or not relative.name.startswith("run_"):
            raise RuntimeError("Slicer acceptance output is outside this task's result directory.")
        report_path = acceptance_folder / "results.json"
        lifecycle_result["slicer_result"] = str(report_path.relative_to(PROJECT.resolve()))
        while time.monotonic() < deadline:
            try:
                progress = read_json(status_path)
            except (OSError, ValueError):
                time.sleep(0.25)
                continue
            if progress.get("timestamp", 0) < started_at:
                time.sleep(0.25)
                continue
            if (progress.get("expected_source") != "ros2_ursim"
                    or Path(progress.get("output_directory", "")).resolve() != acceptance_folder):
                raise RuntimeError("Another or differently sourced Slicer acceptance replaced this run's status.")
            if progress.get("stage") not in ("complete", "failed"):
                time.sleep(0.25)
                continue
            write_json(folder / "slicer_terminal_status.json", progress)
            report = read_json(report_path)
            checks = report.get("checks", {})
            if (progress["stage"] != "complete" or report.get("all_passed") is not True
                    or report.get("expected_source") != "ros2_ursim" or report.get("source") != "ros2_ursim"
                    or report.get("evidence_type") != "real_slicer_ros2_moveit_ursim_virtual_controller_loop"
                    or report.get("simulation_only") is not True or report.get("actual_robot_connected") is not False
                    or Path(report.get("output_directory", "")).resolve() != acceptance_folder
                    or report.get("completed_at", 0) < started_at or not isinstance(checks, dict)
                    or not checks or not all(value is True for value in checks.values())):
                raise RuntimeError("Slicer URSim target/plan/execute/feedback acceptance did not pass; see " + str(report_path))
            lifecycle_result["slicer_check_count"] = len(checks)
            lifecycle_result["slicer_passed"] = True
            passed = True
            return report_path
        raise RuntimeError("Slicer URSim acceptance did not complete within %s seconds; inspect its own preserved results." % timeout)
    finally:
        if not passed:
            lifecycle_result["slicer_passed"] = False
            if acceptance_folder is not None:
                # Stop only this acceptance timer so it cannot issue later target
                # requests after the runner has failed. Keep original results.
                cleanup = """_acceptance = getattr(slicer, 'ur12eAcceptance', None)
_result = {'owned_acceptance_stopped': False}
if _acceptance is not None and str(_acceptance.folder.resolve()) == %r:
    _acceptance.timer.stop()
    _acceptance.stop_recording()
    _result = {'owned_acceptance_stopped': True}
""" % str(acceptance_folder)
                try:
                    write_json(folder / "slicer_runner_cleanup.json", sender(cleanup, timeout=20))
                except Exception as error:
                    lifecycle_result["slicer_cleanup_error"] = str(error)


def archive_validation(original, published, result):
    """Copy a finished local run, including failures, without replacing history."""
    result["original_validation_directory"] = str(original)
    copied = []

    def copy_bytes(source, target):
        # DrvFS may reject Linux utime/chmod even though content is writable.
        # Preserve evidence bytes, not Unix filesystem attributes; original
        # timestamps remain in the JSON records and the original directory.
        if source.is_symlink():
            raise RuntimeError("Unexpected symlink in local URSim evidence: " + str(source))
        if target.exists() or target.is_symlink():
            raise RuntimeError("Refusing to replace existing URSim evidence: " + str(target))
        if source.is_dir():
            target.mkdir()
            for child in source.iterdir():
                copy_bytes(child, target / child.name)
        elif source.is_file():
            shutil.copyfile(source, target)
            if hashlib.sha256(source.read_bytes()).digest() != hashlib.sha256(target.read_bytes()).digest():
                raise RuntimeError("URSim evidence archive hash mismatch: " + str(target))
        else:
            raise RuntimeError("Unsupported local URSim evidence entry: " + str(source))

    if original.is_dir():
        for source in original.iterdir():
            target = published / source.name
            copy_bytes(source, target)
            copied.append({"original": str(source), "published": str(target.relative_to(PROJECT))})
    write_json(published / "archive_provenance.json", {"original_validation_directory": str(original),
               "copied_at": time.time(), "items": copied})
    reports = list(original.glob("*/summary.json"))
    if len(reports) == 1:
        relative = reports[0].relative_to(original)
        result["integration_original_result"] = str(reports[0])
        result["integration_result"] = str((published / relative).relative_to(PROJECT))


def run_checks(args):
    identifier = stamp()
    folder = PROJECT / "output/ursim_validation" / identifier
    folder.mkdir(parents=True)
    result = {"passed": False, "mode": "ursim_controller_integration", "source": "ros2_ursim",
              "run_id": identifier, "started_at": time.time(), "no_slicer": args.no_slicer,
              "control_runtime": str(control_runtime()),
              "boundary": "Local URSim virtual controller; no physical accuracy or hardware safety result."}
    try:
        start_all(backend_only=args.no_slicer)
        workspace = Path(read_json(BACKEND_OWNER).get("workspace", str(workspace_path()))) if BACKEND_OWNER.exists() else workspace_path()
        original = cache_root() / "validation" / identifier
        original.mkdir(parents=True)
        result["original_validation_directory"] = str(original)
        validator = ["/usr/bin/python3", str(PROJECT / "validation/validate_ursim.py"), "--runtime", str(control_runtime()),
                     "--output", str(original), "--container", NAME, "--network", NAME]
        script = """set -e
source /opt/ros/jazzy/setup.bash
source "$1/install/setup.bash"
shift
exec "$@"
"""
        command = ["bash", "-c", script, "ur12e-ursim-validation", str(workspace), *validator]
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID="62", ROS_LOCALHOST_ONLY="1")
        try:
            with (original / "validation.log").open("wb") as stream:
                completed = subprocess.run(command, cwd=REPO, env=environment, stdout=stream, stderr=subprocess.STDOUT)
            result["validation_exit_code"] = completed.returncode
        finally:
            archive_validation(original, folder, result)
        if completed.returncode:
            raise RuntimeError("URSim validation failed; see " + str(folder / "validation.log"))
        reports = list(folder.glob("*/summary.json"))
        if len(reports) != 1:
            raise RuntimeError("URSim validator did not produce one fresh integration summary.")
        report = read_json(reports[0])
        if (report.get("passed") is not True or report.get("source") != "ros2_ursim"
                or report.get("mode") != "ursim_virtual_controller_basic_control"):
            raise RuntimeError("URSim integration summary is absent, failed or has the wrong evidence mode.")
        result["integration_result"] = str(reports[0].relative_to(PROJECT))
        result["integration_original_result"] = str(original / reports[0].relative_to(folder))
        if not args.no_slicer:
            check_slicer_acceptance(folder, result)
        result["passed"] = True
    except BaseException:
        result["error"] = traceback.format_exc()
        raise
    finally:
        if args.stop_after:
            try:
                stop_all(backend_only=args.no_slicer)
                result["stop_passed"] = True
            except Exception as error:
                result["passed"] = False
                result["stop_error"] = str(error)
        result["finished_at"] = time.time()
        write_json(folder / "lifecycle_summary.json", result)
        write_json(PROJECT / "output/ursim_validation/latest.json", result)
        print("URSim lifecycle summary: " + str(folder / "lifecycle_summary.json"), flush=True)
    if not result["passed"]:
        raise RuntimeError("URSim cleanup failed; inspect lifecycle_summary.json.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    start = commands.add_parser("start")
    start_mode = start.add_mutually_exclusive_group()
    start_mode.add_argument("--backend-only", action="store_true")
    start_mode.add_argument("--simulator-only", action="store_true", help="Container and loopback GUI only; no ROS or Slicer")
    stop = commands.add_parser("stop")
    stop.add_argument("--backend-only", action="store_true")
    commands.add_parser("status")
    run = commands.add_parser("run")
    run.add_argument("--no-slicer", action="store_true")
    run.add_argument("--stop-after", action="store_true")
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("Run these URSim entrypoints inside Linux.")
    if args.command == "status":
        print(json.dumps(status(), indent=2))
        return
    if os.geteuid() == 0:
        parser.error("Build/start/validation/stop must use the ordinary project user, not root.")
    import fcntl
    RUNTIME.mkdir(parents=True, exist_ok=True)
    # Share the mock lifecycle lock because both modes use the same file bridge.
    with (RUNTIME / "linux_lifecycle.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Another project lifecycle/validation command is active.")
        if args.command == "start":
            start_all(args.backend_only, args.simulator_only)
        elif args.command == "stop":
            stop_all(args.backend_only)
        else:
            run_checks(args)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR: " + str(error), file=sys.stderr)
        sys.exit(1)
