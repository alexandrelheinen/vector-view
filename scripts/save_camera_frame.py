#!/usr/bin/env python3
"""Capture one frame from the release demo camera topic."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PilImage


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/release_camera")
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    node = Node()
    frame: Image | None = None

    def on_image(msg: Image) -> None:
        nonlocal frame
        frame = msg

    if not node.subscribe(Image, args.topic, on_image):
        raise SystemExit(f"failed to subscribe to {args.topic}")

    end = time.time() + args.timeout
    while frame is None and time.time() < end:
        time.sleep(0.05)

    if frame is None or not frame.data:
        raise SystemExit(f"no image received on {args.topic}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    # Gazebo Harmonic camera publishes RGB_INT8 (3 channels).
    if frame.pixel_format_type == 2 and frame.step >= frame.width * 3:
        mode = "RGB"
        img = PilImage.frombytes(mode, (frame.width, frame.height), bytes(frame.data))
    else:
        # Fallback for unexpected encodings.
        img = PilImage.frombytes("RGB", (frame.width, frame.height), bytes(frame.data)[: frame.width * frame.height * 3])

    img.save(output)
    print(f"saved {output} ({frame.width}x{frame.height})")


if __name__ == "__main__":
    main()
