"""
Deterministic mission-aware thumbnail generator.

The launcher's mission cards used to ship Pillow placeholders that all
looked identical regardless of mission content. This script replaces
each card's PNG with one that's still Pillow-rendered (so CI can run
without a browser) but is *unique per mission* — the terrain bounds and
the per-drone waypoint paths are projected onto the canvas so a 6-drone
ring patrol ships a different image from a single-drone lawnmower.

Usage::

    .venv/bin/python -m simulation.runtime_view.scripts.generate_thumbnails

Each entry in ``simulation/runtime_view/missions.json`` whose
``thumbnail`` field points under ``web/img/`` is regenerated. The
real-screenshot operator workflow (`capture_thumbnails.md`) still
applies when an operator wants the real-flight thumbnails.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict, List, Optional

# Allow `python -m simulation.runtime_view.scripts.generate_thumbnails`
# AND `python generate_thumbnails.py` from inside scripts/.
_THIS_DIR = Path(__file__).resolve().parent
_SIM_DIR = _THIS_DIR.parents[1]
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

import numpy as np  # noqa: E402

WEB_IMG_DIR = _THIS_DIR.parent / "web" / "img"
MISSIONS_JSON = _THIS_DIR.parent / "missions.json"

# Mission id → (mission_kind, drone_count). Drives which waypoint pattern
# the thumbnail visualises; missing entries fall back to single-drone
# patrol so we always render something representative.
_MISSION_KIND_BY_ID = {
    "single": ("patrol", 1),
    "physics": ("patrol", 1),
    "real-log": ("lawnmower", 1),
    "swarm-3": ("patrol", 3),
    "swarm-6": ("patrol", 6),
    "fw": ("lawnmower", 1),
}

CANVAS_W = 640
CANVAS_H = 360
DRONE_PALETTE = [
    (34, 211, 238),
    (46, 212, 122),
    (236, 72, 153),
    (245, 158, 11),
    (139, 92, 246),
    (239, 68, 68),
]


def _project(points: List[np.ndarray]) -> List[tuple]:
    """Project a list of XY points into ``CANVAS_W × CANVAS_H``."""
    arr = np.asarray([p[:2] for p in points], dtype=float)
    if arr.size == 0:
        return []
    x_min, y_min = arr.min(axis=0) - 5.0
    x_max, y_max = arr.max(axis=0) + 5.0
    span_x = max(1e-3, x_max - x_min)
    span_y = max(1e-3, y_max - y_min)
    margin = 32
    out = []
    for x, y in arr:
        u = margin + (x - x_min) / span_x * (CANVAS_W - 2 * margin)
        # Y is inverted (image coords go downward).
        v = CANVAS_H - margin - (y - y_min) / span_y * (CANVAS_H - 2 * margin)
        out.append((float(u), float(v)))
    return out


def _render_thumbnail(mission_id: str, kind: str, n: int,
                       out_path: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont
    from missions import build_mission

    waypoints = build_mission(kind, n)
    img = Image.new("RGB", (CANVAS_W, CANVAS_H), (4, 8, 26))
    draw = ImageDraw.Draw(img)

    # Subtle horizon gradient so the thumbnail doesn't read as a solid block.
    for i in range(CANVAS_H):
        c = int(8 + i / CANVAS_H * 28)
        draw.line([(0, i), (CANVAS_W, i)], fill=(c // 4, c // 3, 26 + c // 4))

    # Per-drone polylines.
    for drone_id, wp_list in waypoints.items():
        color = DRONE_PALETTE[(drone_id - 1) % len(DRONE_PALETTE)]
        pts = _project(wp_list)
        if len(pts) >= 2:
            draw.line(pts, fill=color, width=2)
        for px, py in pts:
            draw.ellipse([px - 4, py - 4, px + 4, py + 4],
                          fill=color, outline=(255, 255, 255))

    # Title overlay.
    label = f"{mission_id}  ({kind} · {n}d)"
    try:
        font = ImageFont.truetype("Arial.ttf", 18)
    except (IOError, OSError):
        font = ImageFont.load_default()
    draw.rectangle([0, 0, CANVAS_W, 28], fill=(4, 8, 26))
    draw.text((12, 6), label, fill=(220, 230, 255), font=font)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(str(out_path), format="PNG")


def regenerate(missions_json: Optional[Path] = None,
               img_dir: Optional[Path] = None) -> Dict[str, Path]:
    """Regenerate every mission card's thumbnail. Returns id → path."""
    missions_json = missions_json or MISSIONS_JSON
    img_dir = img_dir or WEB_IMG_DIR
    catalogue = json.loads(missions_json.read_text(encoding="utf-8"))
    out: Dict[str, Path] = {}
    for entry in catalogue:
        mid = entry.get("id")
        thumb = entry.get("thumbnail", "")
        if not mid or not thumb.startswith("/web/img/"):
            continue
        kind, n = _MISSION_KIND_BY_ID.get(mid, ("patrol", 1))
        out_path = img_dir / Path(thumb).name
        _render_thumbnail(mid, kind, n, out_path)
        out[mid] = out_path
    return out


def main(argv: Optional[list] = None) -> int:
    paths = regenerate()
    for mid, p in paths.items():
        print(f"  {mid}: {p}")
    print(f"\nRegenerated {len(paths)} thumbnails in {WEB_IMG_DIR}/")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
