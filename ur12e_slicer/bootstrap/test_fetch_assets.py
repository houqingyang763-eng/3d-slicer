"""Small resource-integrity checks; no network, ROS or GUI required."""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import fetch_assets


class AssetIntegrityTest(unittest.TestCase):
    def test_verified_cache_is_reused_without_network(self):
        with tempfile.TemporaryDirectory() as temporary:
            cached = Path(temporary) / "archive"
            cached.write_bytes(b"known reference")
            metadata = {"url": "https://unused.invalid/", "sha256": hashlib.sha256(cached.read_bytes()).hexdigest()}
            with patch("urllib.request.urlopen", side_effect=AssertionError("network must remain unused")):
                self.assertEqual(fetch_assets.obtain(cached, metadata, offline=True), cached)

    def test_wrong_digest_is_rejected_and_file_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            cached = Path(temporary) / "archive"
            cached.write_bytes(b"corrupt")
            with self.assertRaisesRegex(RuntimeError, "SHA256 mismatch"):
                fetch_assets.obtain(cached, {"sha256": "0" * 64}, offline=True)
            self.assertEqual(cached.read_bytes(), b"corrupt")

    def test_missing_resource_in_check_only_is_read_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            missing = Path(temporary) / "not-created" / "archive"
            with patch("urllib.request.urlopen", side_effect=AssertionError("network must remain unused")):
                with self.assertRaisesRegex(RuntimeError, "network disabled"):
                    fetch_assets.obtain(missing, {"url": "https://unused.invalid/", "sha256": "0" * 64}, check_only=True)
            self.assertFalse(missing.parent.exists())

    def test_external_slicer_path_is_resolved_explicitly(self):
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "Slicer"
            with patch.dict("os.environ", {"SLICER_EXECUTABLE": str(executable)}):
                self.assertEqual(fetch_assets.resolve_slicer(), executable.resolve())


if __name__ == "__main__":
    unittest.main()
