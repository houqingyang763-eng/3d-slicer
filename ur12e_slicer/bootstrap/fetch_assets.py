#!/usr/bin/env python3
"""Restore pinned public resources; uses Python's standard library only.

--check-only is read-only and never uses the network. --offline may extract
verified cached archives, but never downloads. This script starts no application.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tarfile
import tempfile
import urllib.request
import zipfile

PROJECT = Path(__file__).resolve().parents[1]
REPO = PROJECT.parent


def digest(path, algorithm="sha256"):
    value = hashlib.new(algorithm)
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def verify(path, metadata):
    path = Path(path)
    if not path.is_file():
        return False
    if metadata.get("bytes") is not None and path.stat().st_size != metadata["bytes"]:
        raise RuntimeError("Size mismatch; preserve and inspect the file: " + str(path))
    for algorithm in ("sha256", "sha512"):
        expected = metadata.get(algorithm)
        if expected and digest(path, algorithm) != expected.lower():
            raise RuntimeError(algorithm.upper() + " mismatch; preserve and inspect the file: " + str(path))
    return True


def obtain(path, metadata, offline=False, check_only=False):
    path = Path(path)
    if verify(path, metadata):
        return path
    if offline or check_only:
        raise RuntimeError("Missing verified resource (network disabled): " + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".partial")
    print("Downloading " + metadata["url"], flush=True)
    request = urllib.request.Request(metadata["url"], headers={"User-Agent": "UR12e-Slicer-reproducible-setup/1"})
    try:
        with urllib.request.urlopen(request, timeout=120) as response, temporary.open("wb") as destination:
            shutil.copyfileobj(response, destination, length=1024 * 1024)
        verify(temporary, metadata)
        temporary.replace(path)
    except Exception as error:
        raise RuntimeError("Download/verification failed. Partial file retained at %s: %s" % (temporary, error)) from error
    return path


def description_ready(metadata):
    destination = PROJECT / "assets" / "ur_description"
    if not destination.exists():
        return False
    for relative, expected in metadata["required_files_sha256"].items():
        path = destination / relative
        if not path.is_file():
            return False
        if digest(path) != expected:
            raise RuntimeError("UR description differs from pinned required file: " + str(path))
    return True


def restore_description(offline=False, check_only=False):
    metadata = read_json(PROJECT / "assets" / "description_source.json")
    if description_ready(metadata):
        return "UR description: verified pinned required files"
    if check_only:
        raise RuntimeError("UR description is incomplete. Run bootstrap/fetch_assets.py.")
    destination = PROJECT / "assets" / "ur_description"
    if destination.exists():
        raise RuntimeError("Incomplete UR description preserved at %s; move it aside before restoring." % destination)
    archive = obtain(PROJECT / "downloads" / "ur_description.zip",
                     {"url": metadata["archive"], "sha256": metadata["archive_sha256"]}, offline)
    expected_root = "Universal_Robots_ROS2_Description-" + metadata["commit"] + "/"
    with tempfile.TemporaryDirectory(prefix="ur-description-", dir=PROJECT / "assets") as temporary:
        staged = Path(temporary) / "ur_description"
        staged.mkdir()
        with zipfile.ZipFile(archive) as source:
            for member in source.infolist():
                if not member.filename.startswith(expected_root):
                    raise RuntimeError("Unexpected path in pinned UR description archive")
                relative = Path(member.filename[len(expected_root):])
                if relative.is_absolute() or ".." in relative.parts:
                    raise RuntimeError("Unsafe path in UR description archive")
                target = staged / relative
                if member.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with source.open(member) as incoming, target.open("wb") as outgoing:
                        shutil.copyfileobj(incoming, outgoing)
        staged.rename(destination)
    if not description_ready(metadata):
        raise RuntimeError("Restored UR description is incomplete")
    return "UR description: restored fixed commit " + metadata["commit"]


def resolve_slicer():
    """The shared launcher contract: explicit executable, then managed path."""
    override = os.environ.get("SLICER_EXECUTABLE")
    return Path(override).expanduser().resolve() if override else PROJECT / ".local" / "slicer" / "Slicer"


def slicer_ready(metadata):
    executable = resolve_slicer()
    if not executable.is_file():
        return False
    if not os.access(executable, os.X_OK):
        raise RuntimeError("Slicer launcher is not executable: " + str(executable))
    if os.environ.get("SLICER_EXECUTABLE"):
        # An explicit external installation is operator-managed. Its version is
        # reported as unverified; never describe it as our checksum-verified build.
        return True
    receipt = executable.parent / ".ur12e-install.json"
    if not receipt.is_file():
        return False
    saved = read_json(receipt)
    if saved.get("archive_sha256") != metadata["sha256"] or saved.get("version") != metadata["version"]:
        raise RuntimeError("Slicer receipt does not match the pinned version")
    if digest(executable) != saved.get("launcher_sha256"):
        raise RuntimeError("Slicer launcher differs from its verified installation receipt")
    return True


def restore_slicer(offline=False, check_only=False):
    metadata = read_json(PROJECT / "assets" / "slicer_linux_source.json")
    if slicer_ready(metadata):
        qualifier = "external executable; version/hash operator-managed" if os.environ.get("SLICER_EXECUTABLE") else "verified installation receipt"
        return "Slicer: %s (%s)" % (resolve_slicer(), qualifier)
    if os.environ.get("SLICER_EXECUTABLE"):
        raise RuntimeError("SLICER_EXECUTABLE does not point to a usable executable: " + str(resolve_slicer()))
    if check_only:
        raise RuntimeError("Pinned Linux Slicer is missing. Run bootstrap/install_linux.sh or set SLICER_EXECUTABLE.")
    destination = PROJECT / ".local" / "slicer"
    if destination.exists():
        raise RuntimeError("Incomplete Slicer directory preserved at %s; move it aside before restoring." % destination)
    archive = obtain(PROJECT / "downloads" / metadata["filename"], metadata, offline)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="slicer-", dir=destination.parent) as temporary:
        with tarfile.open(archive, "r:gz") as source:
            # Ubuntu 24.04 supplies Python 3.12 and its safe data extraction filter.
            source.extractall(temporary, filter="data")
        roots = list(Path(temporary).iterdir())
        if len(roots) != 1 or not (roots[0] / "Slicer").is_file():
            raise RuntimeError("Unexpected Slicer archive layout")
        receipt = {"version": metadata["version"], "archive_sha256": metadata["sha256"],
                   "archive_sha512": metadata["sha512"], "launcher_sha256": digest(roots[0] / "Slicer")}
        (roots[0] / ".ur12e-install.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
        roots[0].rename(destination)
    if not slicer_ready(metadata):
        raise RuntimeError("Slicer restoration did not pass its receipt check")
    return "Slicer: restored " + str(resolve_slicer())


def restore_mr(offline=False, check_only=False):
    metadata = next(value for value in read_json(REPO / "data" / "sources.json") if value["file"] == "MR-head.nrrd")
    obtain(REPO / "data" / metadata["file"], {"url": metadata["source"], "sha256": metadata["sha256"], "bytes": metadata["bytes"]}, offline, check_only)
    return "MR-head.nrrd: verified source SHA256"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--skip-slicer", action="store_true")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--slicer-only", action="store_true")
    group.add_argument("--description-only", action="store_true")
    args = parser.parse_args()
    try:
        if not args.slicer_only:
            print(restore_description(args.offline, args.check_only))
            if not args.description_only:
                print(restore_mr(args.offline, args.check_only))
        if not args.skip_slicer and not args.description_only:
            print(restore_slicer(args.offline, args.check_only))
    except (OSError, ValueError, RuntimeError, tarfile.TarError, zipfile.BadZipFile) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
