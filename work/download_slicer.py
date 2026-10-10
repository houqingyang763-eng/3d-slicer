"""Download the official Windows stable package, retaining provenance."""
from pathlib import Path
import hashlib
import json
import sys
import time
import urllib.request

sys.stdout.reconfigure(encoding="utf-8")
root = Path(__file__).resolve().parents[1]
folder = root / "work" / "downloads"
folder.mkdir(parents=True, exist_ok=True)
url = "https://download.slicer.org/bitstream/6aa1db04ce9de556d30112bb"
target = folder / "Slicer-5.12.4-win-amd64.exe"
with urllib.request.urlopen(url, timeout=60) as response:
    total = int(response.headers["Content-Length"])
    metadata = {"source_url": url, "resolved_url": response.url, "expected_bytes": total}
    received = 0
    last = time.monotonic()
    partial = target.with_suffix(".exe.part")
    with partial.open("wb") as stream:
        while chunk := response.read(1024 * 1024):
            stream.write(chunk)
            received += len(chunk)
            if time.monotonic() - last > 15:
                print(f"Downloaded {received / 1024**2:.1f}/{total / 1024**2:.1f} MiB", flush=True)
                last = time.monotonic()
    if received != total:
        raise RuntimeError(f"Incomplete download: {received} of {total} bytes")
    partial.replace(target)
metadata["actual_bytes"] = target.stat().st_size
metadata["sha512"] = hashlib.sha512(target.read_bytes()).hexdigest()
(folder / "slicer_package.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
print(f"READY: {target}", flush=True)
