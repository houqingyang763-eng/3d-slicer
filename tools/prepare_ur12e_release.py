#!/usr/bin/env python3
"""Check or copy the explicit weekly publish allowlist without changing Git.

Run from any directory with Python 3.10+:
  python3 tools/prepare_ur12e_release.py
  python3 tools/prepare_ur12e_release.py --print-files
  python3 tools/prepare_ur12e_release.py --copy-to /tmp/ur12e-weekly-package

This is a source-package check, not a Linux runtime test. It does not install
dependencies, publish the internal presentation, rewrite history, or stage Git.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import sys


REPO = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = REPO / "ur12e_slicer" / "publish_manifest.json"
FORBIDDEN_PARTS = {
    "node_modules", "__pycache__", ".venv", ".local", "runtime", "downloads",
    ".codex-finalizer", "build", "install", "log",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_path(relative: str) -> Path:
    value = PurePosixPath(relative)
    if value.is_absolute() or ".." in value.parts or "\\" in relative or ":" in relative:
        raise ValueError(f"Not a repository-relative POSIX path: {relative}")
    if any(part in FORBIDDEN_PARTS or part.startswith(".chart-data-") for part in value.parts):
        raise ValueError(f"Generated / installed directory is forbidden: {relative}")
    if value.suffix.lower() in {".pyc", ".pyo", ".pptx", ".pdf", ".mrb", ".msi", ".exe"}:
        raise ValueError(f"Internal presentation or binary installation is forbidden: {relative}")
    if value.parts[:2] in {("ur12e_slicer", "output"), ("work", "bridge")}:
        raise ValueError(f"Live output or local communication state is forbidden: {relative}")
    target = REPO.joinpath(*value.parts)
    for ancestor in (target, *target.parents):
        if ancestor == REPO:
            break
        if ancestor.is_symlink() or (hasattr(ancestor, "is_junction") and ancestor.is_junction()):
            raise ValueError(f"Symlink / junction is forbidden: {relative}")
    if not target.is_file() or not target.resolve().is_relative_to(REPO):
        raise ValueError(f"Missing or out-of-tree file: {relative}")
    return target


def verify_reference_package(provenance_path: Path) -> int:
    payload = json.loads(provenance_path.read_text(encoding="utf-8"))
    for item in payload["files"]:
        published = validate_path(item["published_path"])
        if sha256(published) != item["published_sha256"]:
            raise ValueError(f"Reference checksum mismatch: {item['published_path']}")
    reference = provenance_path.parent
    offline = json.loads((reference / "offline/truth.json").read_text(encoding="utf-8"))
    offline_run = json.loads((reference / "offline/run.json").read_text(encoding="utf-8"))
    canonical = json.dumps(offline["truth"], sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False, allow_nan=False).encode("utf-8")
    if hashlib.sha256(canonical).hexdigest() != offline["sha256"] or offline["sha256"] != offline_run["truth_sha256"]:
        raise ValueError("Offline independent truth hash does not match its run")
    integration_truth = json.loads((reference / "moveit_mock/truth.json").read_text(encoding="utf-8"))
    integration_run = json.loads((reference / "moveit_mock/run.json").read_text(encoding="utf-8"))
    canonical = json.dumps(integration_truth, sort_keys=True, allow_nan=False).encode("utf-8")
    if hashlib.sha256(canonical).hexdigest() != integration_run["truth_sha256"]:
        raise ValueError("MoveIt/mock independent truth hash does not match its run")
    return len(payload["files"])


def verify_reference() -> int:
    packages = sorted((REPO / "ur12e_slicer/reference_results").glob("*/provenance.json"))
    if not packages:
        raise ValueError("No provenance-recorded reference package found")
    return sum(verify_reference_package(path) for path in packages)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--print-files", action="store_true", help="Print one allowlisted path per line")
    parser.add_argument("--copy-to", type=Path, help="Copy to a new or empty directory; never delete existing files")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    files = manifest["files"]
    if files != sorted(set(files)):
        raise ValueError("The allowlist must be sorted and contain no duplicates")
    rows = []
    for relative in files:
        source = validate_path(relative)
        rows.append({"path": relative, "bytes": source.stat().st_size, "sha256": sha256(source)})
    checked_reference = verify_reference()
    if args.copy_to:
        destination = args.copy_to.resolve()
        if destination == REPO or destination.is_relative_to(REPO / "ur12e_slicer"):
            raise ValueError("The package destination must not be inside ur12e_slicer or overwrite the repository")
        if destination.exists() and (not destination.is_dir() or any(destination.iterdir())):
            raise ValueError("The package destination must be new or empty")
        destination.mkdir(parents=True, exist_ok=True)
        for row in rows:
            source = REPO / row["path"]
            target = destination / row["path"]
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            if sha256(target) != row["sha256"]:
                raise ValueError(f"Copied file checksum mismatch: {row['path']}")
        (destination / "package_inventory.json").write_text(
            json.dumps({"schema_version": 1, "source_manifest": "ur12e_slicer/publish_manifest.json", "files": rows},
                       ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.print_files:
        print("\n".join(files))
    else:
        print(json.dumps({"checked_files": len(rows), "total_bytes": sum(row["bytes"] for row in rows),
                          "checked_reference_files": checked_reference,
                          "copied_to": str(args.copy_to.resolve()) if args.copy_to else None,
                          "git_modified": False}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError) as error:
        print(f"Package check failed: {error}", file=sys.stderr)
        raise SystemExit(1)
