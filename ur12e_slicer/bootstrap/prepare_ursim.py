#!/usr/bin/env python3
"""Fetch verified official URSim assets without starting any application."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent


def prepare():
    if sys.platform != "linux" or os.geteuid() == 0:
        raise RuntimeError("Run preparation as an ordinary Linux user with access to local Docker.")
    if os.environ.get("DOCKER_HOST", "unix:///var/run/docker.sock") != "unix:///var/run/docker.sock":
        raise RuntimeError("Only the local /var/run/docker.sock Docker endpoint is allowed.")
    docker = ["docker", "--host", "unix:///var/run/docker.sock"]
    source = json.loads((PROJECT / "assets/ursim_source.json").read_text())
    key = hashlib.sha256(str(REPO.resolve()).encode()).hexdigest()[:12]
    cache = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "ur12e_slicer" / key / "ursim"
    downloads = PROJECT / "downloads/ursim"
    downloads.mkdir(parents=True, exist_ok=True)
    for name in ("programs", "urcaps"):
        (cache / name).mkdir(parents=True, exist_ok=True)

    def fetch(spec, destination):
        if destination.exists() and hashlib.sha256(destination.read_bytes()).hexdigest() == spec["sha256"]:
            return destination.read_bytes()
        # An unsuccessful transfer never replaces the previously verified asset.
        with urllib.request.urlopen(spec["url"], timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != spec["sha256"]:
            raise RuntimeError("SHA256 mismatch for " + spec["url"])
        temporary = destination.with_suffix(destination.suffix + ".part")
        temporary.write_bytes(data)
        temporary.replace(destination)
        return data

    cap = source["external_control"]
    cap_data = fetch(cap, downloads / cap["filename"])
    if len(cap_data) != cap["bytes"]:
        raise RuntimeError("External Control asset size mismatch.")
    (cache / "urcaps" / cap["filename"]).write_bytes(cap_data)
    template = fetch(source["program_template"], downloads / "external_control.urp")
    program = cache / "programs/external_control.urp"
    if not program.exists():
        root = ET.fromstring(gzip.decompress(template))
        for name in ("robotSerialNumber", "crcValue"):
            root.attrib.pop(name, None)
        for child in list(root):
            if child.tag == "kinematics":
                root.remove(child)
        root.set("name", "external_control")
        program.write_bytes(gzip.compress(ET.tostring(root, encoding="utf-8"), mtime=0))

    image = source["image"]
    reference = image["repository"] + "@" + image["digest"]
    inspected = subprocess.run(docker + ["image", "inspect", reference], capture_output=True, text=True)
    if inspected.returncode:
        subprocess.run(docker + ["pull", "--platform", image["platform"], reference], check=True)
    result = json.loads(subprocess.check_output(docker + ["image", "inspect", reference], text=True))[0]
    if reference not in result.get("RepoDigests", []):
        raise RuntimeError("Installed image RepoDigest does not match the selected official image.")
    inventory = {"prepared": True, "project_key": key, "cache": str(cache), "image": reference,
                 "image_id": result["Id"], "external_control": cap["version"],
                 "program_gui_configuration_required": True,
                 "instruction": "Use UR12-generated installation; set External Control Host IP 192.168.62.1, port 50002, then save the program and installation in PolyScope."}
    (cache / "prepared.json").write_text(json.dumps(inventory, indent=2), encoding="utf-8")
    print(json.dumps(inventory, indent=2))


if __name__ == "__main__":
    try:
        prepare()
    except Exception as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
