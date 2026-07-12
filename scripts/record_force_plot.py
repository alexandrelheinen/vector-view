#!/usr/bin/env python3
"""Record filtered and raw force magnitudes from Vector View contact topics."""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node


def force_magnitude(message: Contacts) -> float:
    """Return the largest body-1 force magnitude in one contact message."""
    peak = 0.0
    for contact in message.contact:
        for wrench in contact.wrench:
            force = wrench.body_1_wrench.force
            magnitude = (force.x * force.x + force.y * force.y + force.z * force.z) ** 0.5
            peak = max(peak, magnitude)
    return peak


def simple_filter(values: list[float], alpha: float = 0.2) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    for value in values[1:]:
        out.append(alpha * value + (1.0 - alpha) * out[-1])
    return out


def draw_plot(label: str, times: list[float], raw: list[float], filtered: list[float], output: Path) -> None:
    from PIL import Image, ImageDraw

    width, height = 504, 312
    margin_left, margin_right, margin_top, margin_bottom = 48, 16, 28, 36
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    draw.text((margin_left, 6), label, fill="black")
    draw.text((margin_left + plot_width // 2 - 24, height - 18), "time (s)", fill="black")

    x0 = margin_left
    y0 = margin_top
    x1 = margin_left + plot_width
    y1 = margin_top + plot_height
    draw.rectangle((x0, y0, x1, y1), outline="#666666")

    max_force = max(max(raw, default=1.0), max(filtered, default=1.0), 1.0)
    max_force *= 1.15
    max_time = max(times[-1] - times[0], 0.25) if len(times) >= 2 else 1.0
    t_start = times[0] if times else 0.0

    def to_xy(t: float, value: float) -> tuple[float, float]:
        x = x0 + ((t - t_start) / max_time) * plot_width
        y = y1 - (value / max_force) * plot_height
        return x, y

    for fraction in (0.0, 0.5, 1.0):
        x = x0 + fraction * plot_width
        y = y1 - fraction * plot_height
        draw.line((x, y0, x, y1), fill="#e6e6e6", width=1)
        draw.line((x0, y, x1, y), fill="#e6e6e6", width=1)
        draw.text((x - 8, y1 + 4), f"{fraction * max_time:.1f}", fill="#555555")
        draw.text((2, y - 5), f"{fraction * max_force:.0f}", fill="#555555")

    if len(times) >= 2:
        raw_points = [to_xy(t, v) for t, v in zip(times, raw)]
        filt_points = [to_xy(t, v) for t, v in zip(times, filtered)]
        for i in range(len(raw_points) - 1):
            draw.line(raw_points[i] + raw_points[i + 1], fill="#b0b0b0", width=2)
        for i in range(len(filt_points) - 1):
            draw.line(filt_points[i] + filt_points[i + 1], fill="#1e88e5", width=3)

    draw.text((x1 - 70, y0 + 8), "original", fill="#888888")
    draw.text((x1 - 70, y0 + 24), "filtered", fill="#1e88e5")
    draw.text((8, y0 + plot_height // 2), "Force (N)", fill="black")
    output.parent.mkdir(parents=True, exist_ok=True)
    img.save(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()

    node = Node()
    lock = threading.Lock()
    latest_force = 0.0
    messages = 0

    def on_contacts(message: Contacts) -> None:
        nonlocal latest_force, messages
        with lock:
            latest_force = force_magnitude(message)
            messages += 1

    if not node.subscribe(Contacts, args.topic, on_contacts):
        raise SystemExit(f"failed to subscribe to {args.topic}")

    times: list[float] = []
    raw: list[float] = []
    start = time.monotonic()
    while (elapsed := time.monotonic() - start) < args.seconds:
        with lock:
            magnitude = latest_force
        times.append(elapsed)
        raw.append(magnitude)
        time.sleep(1.0 / 25.0)

    nonzero_samples = sum(value > 1e-3 for value in raw)
    if messages == 0 or nonzero_samples < 2:
        raise SystemExit(
            f"no real force trace on {args.topic}: "
            f"messages={messages}, nonzero_samples={nonzero_samples}"
        )

    filtered = simple_filter(raw)
    output = Path(args.output)
    draw_plot(args.label, times, raw, filtered, output)
    output.with_suffix(".json").write_text(
        json.dumps(
            {
                "topic": args.topic,
                "samples": len(raw),
                "messages": messages,
                "nonzero_samples": nonzero_samples,
                "duration_seconds": times[-1],
                "max_force_newtons": max(raw),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
