#!/usr/bin/env python3
"""Record filtered and raw force magnitudes from Vector View contact topics."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path


def gz_echo(topic: str, count: int) -> str:
    proc = subprocess.run(
        ["timeout", "2", "gz", "topic", "-e", "-t", topic, "-n", str(count)],
        capture_output=True,
        text=True,
        check=False,
    )
    return proc.stdout


def parse_forces(text: str) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    blocks = re.split(r"\n(?=header \{)", text)

    force_pattern = re.compile(
        r"body_1_wrench\s*\{.*?force\s*\{([^}]*)\}",
        flags=re.S,
    )

    for block in blocks:
        sec_match = re.search(r"sec:\s*([0-9]+)", block)
        nsec_match = re.search(r"nsec:\s*([0-9]+)", block)
        if not sec_match:
            continue
        current_time = float(sec_match.group(1))
        if nsec_match:
            current_time += float(nsec_match.group(1)) * 1e-9

        peak = 0.0
        for force_block in force_pattern.findall(block):
            fx = float(m.group(1)) if (m := re.search(r"x:\s*([-0-9.eE]+)", force_block)) else 0.0
            fy = float(m.group(1)) if (m := re.search(r"y:\s*([-0-9.eE]+)", force_block)) else 0.0
            fz = float(m.group(1)) if (m := re.search(r"z:\s*([-0-9.eE]+)", force_block)) else 0.0
            magnitude = (fx * fx + fy * fy + fz * fz) ** 0.5
            peak = max(peak, magnitude)

        if peak > 1e-3:
            samples.append((current_time, peak))

    return samples


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

    chunks: list[str] = []
    end = time.time() + args.seconds
    misses = 0
    while time.time() < end:
        chunk = gz_echo(args.topic, 1)
        if chunk.strip():
            chunks.append(chunk)
            misses = 0
        else:
            misses += 1
            if misses >= 5 and not chunks:
                break
        time.sleep(0.08)

    samples = parse_forces("\n".join(chunks))
    if len(samples) < 2:
        # Emit a minimal placeholder plot instead of aborting capture.
        output = Path(args.output)
        draw_plot(args.label, [0.0, 1.0], [0.0, 0.0], [0.0, 0.0], output)
        output.with_suffix(".json").write_text(
            json.dumps({"topic": args.topic, "samples": len(samples), "warning": "no data"}, indent=2)
        )
        return

    samples.sort(key=lambda sample: sample[0])
    t0 = samples[0][0]
    times = [sample[0] - t0 for sample in samples]
    raw = [sample[1] for sample in samples]
    filtered = simple_filter(raw)
    output = Path(args.output)
    draw_plot(args.label, times, raw, filtered, output)
    output.with_suffix(".json").write_text(
        json.dumps({"topic": args.topic, "samples": len(samples)}, indent=2)
    )


if __name__ == "__main__":
    main()
