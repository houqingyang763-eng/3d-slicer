#!/usr/bin/env python3
"""Atomically copy verified, official-index ROS packages into the apt cache."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import time

project = Path(__file__).resolve().parents[1]
manifest = json.loads((project / "runtime/apt_package_uris.json").read_text(encoding="utf-8-sig"))
source = project / "downloads/apt_cache"
destination = Path("/var/cache/apt/archives")
if os.name != "posix" or not Path("/etc/os-release").exists():
    raise SystemExit("Run this script inside the dedicated Ubuntu distribution.")
report = {"started_at": time.time(), "copied": [], "already_verified": [], "not_yet_downloaded": [], "errors": []}

def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()

for package in manifest["packages"]:
    if package["host"] != "packages.ros.org":
        continue
    name = package["cache_filename"]
    if Path(name).name != name or not name.endswith(".deb"):
        raise SystemExit("Invalid cache filename in the pinned package manifest.")
    candidate, cached = source / name, destination / name
    if not candidate.exists():
        report["not_yet_downloaded"].append(name)
        continue
    try:
        if candidate.stat().st_size != package["size"] or digest(candidate) != package["sha256"]:
            raise ValueError("Source size or SHA256 differs from official apt index")
        if cached.exists() and cached.stat().st_size == package["size"] and digest(cached) == package["sha256"]:
            report["already_verified"].append(name)
            continue
        fd, temporary = tempfile.mkstemp(prefix=".ur12e-prefetch-", suffix=".tmp", dir=destination)
        os.close(fd)
        try:
            shutil.copyfile(candidate, temporary)
            os.chmod(temporary, 0o644)
            if digest(Path(temporary)) != package["sha256"]:
                raise ValueError("Copied file SHA256 mismatch")
            os.replace(temporary, cached)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        report["copied"].append(name)
    except (OSError, ValueError) as exc:
        report["errors"].append({"file": name, "error": str(exc)})

report["finished_at"] = time.time()
(project / "runtime/apt_cache_import.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps({key: len(report[key]) for key in ("copied", "already_verified", "not_yet_downloaded", "errors")}))
raise SystemExit(1 if report["errors"] else 0)
