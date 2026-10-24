#!/usr/bin/env python3
"""Resolve the simulation dependencies using python-apt without changing apt/dpkg.

This reads the existing trusted apt indexes. It never updates indexes, takes a
package-manager lock, writes package selections, or installs/removes packages.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from urllib.parse import urlparse

import apt
import apt_pkg


DEFAULT_PACKAGES = [
    "ros-jazzy-desktop", "ros-dev-tools", "ros-jazzy-moveit", "ros-jazzy-ur",
    "ros-jazzy-ros2controlcli", "ros-jazzy-tf2-tools",
]
ALLOWED_HOSTS = {"packages.ros.org", "archive.ubuntu.com", "security.ubuntu.com"}


def status_fingerprint():
    stat = Path("/var/lib/dpkg/status").stat()
    return {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=(
        Path(__file__).resolve().parent.parent / "runtime" / "apt_package_uris.json"))
    parser.add_argument("--packages", nargs="+", default=DEFAULT_PACKAGES)
    args = parser.parse_args()
    before = status_fingerprint()
    cache = apt.Cache()
    for name in args.packages:
        if name not in cache or cache[name].candidate is None:
            raise RuntimeError(f"Package has no indexed candidate: {name}")
        cache[name].mark_install(auto_fix=True, auto_inst=True, from_user=True)
    if cache.broken_count:
        raise RuntimeError(f"Read-only dependency resolution has {cache.broken_count} broken packages")

    packages = []
    for package in cache.get_changes():
        if package.marked_delete:
            raise RuntimeError(f"Resolution would remove a package: {package.name}")
        if not (package.marked_install or package.marked_upgrade or package.marked_downgrade
                or package.marked_reinstall):
            continue
        candidate = package.candidate
        if candidate is None or not candidate.downloadable:
            raise RuntimeError(f"No downloadable candidate: {package.name}")
        uri = candidate.uri
        parsed = urlparse(uri)
        if (parsed.scheme not in ("http", "https") or parsed.hostname not in ALLOWED_HOSTS
                or parsed.username or parsed.password):
            raise RuntimeError(f"Candidate is not from an allowed official host: {package.name}")
        trusted_origins = [origin for origin in candidate.origins if origin.trusted
                           and origin.site == parsed.hostname
                           and origin.origin in ("ROS", "Ubuntu")]
        if not trusted_origins:
            raise RuntimeError(f"Candidate lacks a trusted official apt origin: {package.name}")
        sha256 = candidate.sha256.lower()
        if not re.fullmatch(r"[0-9a-f]{64}", sha256) or candidate.size <= 0:
            raise RuntimeError(f"Candidate lacks SHA256/size: {package.name}")
        cache_filename = "{}_{}_{}.deb".format(
            package.name.split(":", 1)[0], apt_pkg.quote_string(candidate.version, ":"),
            candidate.architecture)
        packages.append({
            "package": package.name, "version": candidate.version,
            "architecture": candidate.architecture, "uri": uri,
            "size": candidate.size, "sha256": sha256,
            "cache_filename": cache_filename, "host": parsed.hostname,
            "trusted_origins": [{"origin": origin.origin, "label": origin.label,
                                 "site": origin.site, "trusted": origin.trusted}
                                for origin in trusted_origins],
        })
    after = status_fingerprint()
    if before != after:
        raise RuntimeError("dpkg status changed while reading. Retry after the current apt operation; output was not replaced.")
    packages.sort(key=lambda package: (package["host"], package["package"]))
    summary = {
        "package_count": len(packages),
        "total_bytes": sum(package["size"] for package in packages),
        "ros_package_count": sum(package["host"] == "packages.ros.org" for package in packages),
        "ros_bytes": sum(package["size"] for package in packages
                         if package["host"] == "packages.ros.org"),
    }
    result = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "resolver": "python-apt in-memory marks only; no commit/update/locks",
        "requested_packages": args.packages, "dpkg_status": after,
        "summary": summary, "packages": packages,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(f"{args.output.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(args.output)
    finally:
        temporary.unlink(missing_ok=True)
    print(json.dumps({"output": str(args.output), **summary}))


if __name__ == "__main__":
    main()
