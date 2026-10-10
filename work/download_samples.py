"""Fetch two official Slicer teaching datasets into the project."""
from pathlib import Path
import hashlib
import json
import sys
import urllib.request

sys.stdout.reconfigure(encoding="utf-8")
root = Path(__file__).resolve().parents[1]
folder = root / "data"
folder.mkdir(exist_ok=True)
samples = {
    "MR-head.nrrd": "cc211f0dfd9a05ca3841ce1141b292898b2dd2d3f08286affadf823a7e58df93",
    "CT-brain.nrrd": "6a5b6caccb76576a863beb095e3bfb910c50ca78f4c9bf043aa42f976cfa53d1",
}
manifest = []
for name, checksum in samples.items():
    url = f"https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/{checksum}"
    target = folder / name
    if not target.exists():
        print(f"Downloading {name}", flush=True)
        with urllib.request.urlopen(url, timeout=60) as response:
            partial = target.with_suffix(".part")
            with partial.open("wb") as output:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
            partial.replace(target)
    actual = hashlib.sha256(target.read_bytes()).hexdigest()
    if actual != checksum:
        raise RuntimeError(f"Incorrect checksum: {name}")
    manifest.append({"file": name, "source": url, "sha256": actual, "bytes": target.stat().st_size})
    print(f"READY: {name} ({target.stat().st_size} bytes)", flush=True)
(folder / "sources.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
