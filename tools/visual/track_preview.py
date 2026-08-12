#!/usr/bin/env python3
"""Track top-down preview (issue #42).

Renders a shipped track manifest as a deterministic SVG: centreline, racing surface and
runoff bands, barriers, checkpoints/sectors/grid/pit markers, and the 2.5D profile
(elevation/banking/kerb) as a side strip. Pure stdlib, no assets required.

Usage: track_preview.py data/tracks/chicane.track.json [out.svg]
Prints the SVG path to stdout when no output file is given.
"""

import json
import math
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    path = sys.argv[1]
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    out_path = sys.argv[2] if len(sys.argv) > 2 else None

    nodes = doc["route"]["nodes"]
    xs = [n["x"] for n in nodes]
    ys = [n["y"] for n in nodes]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    pad = 20.0
    scale = 6.0  # px per metre

    def px(x: float, y: float) -> tuple[float, float]:
        return ((x - min_x) * scale + pad, (y - min_y) * scale + pad)

    svg = []
    svg.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{int((max_x - min_x) * scale + 2 * pad)}" '
        f'height="{int((max_y - min_y) * scale + 2 * pad)}">'
    )

    def poly(points, fill, opacity=1.0, stroke="none"):
        pts = " ".join(f"{a:.1f},{b:.1f}" for a, b in points)
        svg.append(
            f'<polygon points="{pts}" fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}"/>'
        )

    # Surface bands: racing surface then runoff, per segment.
    closed = doc["route"].get("closed", True)
    segs = len(nodes) if closed else len(nodes) - 1
    for i in range(segs):
        a = nodes[i]
        b = nodes[(i + 1) % len(nodes)] if closed else nodes[i + 1]
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        nx, ny = -dy / length, dx / length
        hw = a.get("halfWidth", 4.0)
        runoff = a.get("runoffHalfWidth", hw)
        # Runoff band (kerb zone).
        for side in (1, -1):
            q = [
                px(a["x"] + nx * hw * side, a["y"] + ny * hw * side),
                px(b["x"] + nx * hw * side, b["y"] + ny * hw * side),
                px(b["x"] + nx * runoff * side, b["y"] + ny * runoff * side),
                px(a["x"] + nx * runoff * side, a["y"] + ny * runoff * side),
            ]
            poly(q, "#d9c9a3", 0.6)
        # Racing surface.
        for side in (1, -1):
            q = [
                px(a["x"] + nx * hw * side, a["y"] + ny * hw * side),
                px(b["x"] + nx * hw * side, b["y"] + ny * hw * side),
                px(b["x"], b["y"]),
                px(a["x"], a["y"]),
            ]
            poly(q, "#3a3a3a")

    # Barriers at the runoff edge.
    for i in range(segs):
        a = nodes[i]
        b = nodes[(i + 1) % len(nodes)] if closed else nodes[i + 1]
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        nx, ny = -dy / length, dx / length
        runoff = a.get("runoffHalfWidth", a.get("halfWidth", 4.0))
        for side in (1, -1):
            p1 = px(a["x"] + nx * runoff * side, a["y"] + ny * runoff * side)
            p2 = px(b["x"] + nx * runoff * side, b["y"] + ny * runoff * side)
            svg.append(
                f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" '
                f'y2="{p2[1]:.1f}" stroke="#cc4444" stroke-width="1"/>'
            )

    # Centreline.
    line = " ".join(f"{px(n['x'], n['y'])[0]:.1f},{px(n['x'], n['y'])[1]:.1f}" for n in nodes)
    svg.append(f'<polyline points="{line}" fill="none" stroke="#eeeeee" stroke-width="1.5"/>')

    # Checkpoints.
    for i, cp in enumerate(doc.get("checkpoints", [])):
        x, y = px(cp["x"], cp["y"])
        svg.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3" fill="#44cc44" '
            f'stroke="none"><title>gate {i}</title></circle>'
        )

    # Sectors.
    for i, s in enumerate(doc.get("sectors", [])):
        x, y = px(s["x"], s["y"])
        svg.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3" fill="#44aaff" '
            f'stroke="none"><title>sector {i}</title></circle>'
        )

    # Grid slots.
    for i, g in enumerate(doc.get("grid", [])):
        x, y = px(g["x"], g["y"])
        svg.append(
            f'<rect x="{x - 2:.1f}" y="{y - 2:.1f}" width="4" height="4" fill="#ffaa00" '
            f'stroke="none"><title>grid {i}</title></rect>'
        )

    # Pit gates.
    pit = doc.get("pit", {})
    for key, colour in (("entry", "#ff66aa"), ("exit", "#66ffaa"), ("speedLine", "#ffff66")):
        gate = pit.get(key)
        if gate:
            x, y = px(gate["x"], gate["y"])
            svg.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3" fill="{colour}" '
                f'stroke="none"><title>pit {key}</title></circle>'
            )

    svg.append("</svg>")

    content = "\n".join(svg)
    if out_path:
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"wrote {out_path}")
    else:
        print(content)
    return 0


if __name__ == "__main__":
    sys.exit(main())
