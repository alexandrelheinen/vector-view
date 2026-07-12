#!/usr/bin/env python3
"""Unit tests for release video verification thresholds."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from verify_release_video import MIN_DURATION_SECONDS, MIN_FRAME_COUNT, verify  # noqa: E402


class ReleaseVideoVerifierTests(unittest.TestCase):
    def test_rejects_short_clip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            video = Path(tmp_dir) / "short.mp4"
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-f",
                    "lavfi",
                    "-i",
                    "color=c=black:s=1280x720:d=0.04",
                    "-frames:v",
                    "2",
                    str(video),
                ],
                check=True,
            )
            with self.assertRaises(SystemExit):
                verify(video)

    def test_thresholds_are_sane(self) -> None:
        self.assertGreaterEqual(MIN_DURATION_SECONDS, 10.0)
        self.assertGreaterEqual(MIN_FRAME_COUNT, 200)


if __name__ == "__main__":
    unittest.main()
