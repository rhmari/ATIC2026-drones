from __future__ import annotations

from html import escape
from pathlib import Path

import numpy as np

from .core import Rule
from .scene import Scene, Zone


# The optimiser uses stable internal REQ-* identifiers. The public maps show
# only the human-readable scenario abstractions, not a private rulebook.
RULE_PLOT_NAMES: dict[str, tuple[str, ...]] = {
    "REQ-PROX-01": ("Crowd standoff",),
    "REQ-PLT-03": ("Emergency-area exclusion",),
    "REQ-PROX-06": ("Operational-volume containment",),
    "REQ-ZONE-03": ("Altitude ceiling",),
    "REQ-ZONE-04": ("No-fly-zone exclusion",),
    "REQ-EQP-03": ("Low-battery safe landing",),
    "REQ-EQP-07": ("Maximum speed",),
    "REQ-EQP-04": ("Night light",),
    "REQ-EQP-12": ("Quiet-zone noise limit",),
}


def _hex_opacity(hex_color: str, opacity: float) -> str:
    color = hex_color.lstrip("#")
    if len(color) != 6:
        return hex_color
    r = int(color[0:2], 16)
    g = int(color[2:4], 16)
    b = int(color[4:6], 16)
    return f"rgba({r},{g},{b},{opacity})"


class SvgCanvas:
    def __init__(self, scene: Scene, title: str):
        x_max, y_max, _ = scene.env
        self.margin = 48
        self.legend_w = 360
        self.scale = min(920.0 / x_max, 660.0 / y_max)
        self.map_w = x_max * self.scale
        self.map_h = y_max * self.scale
        self.width = int(self.margin * 2 + self.map_w + self.legend_w)
        self.height = int(self.margin * 2 + max(self.map_h, 560.0))
        self.scene = scene
        self.parts: list[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" '
            f'viewBox="0 0 {self.width} {self.height}">',
            "<style>",
            "text{font-family:Inter,Arial,sans-serif;fill:#1f2937}",
            ".small{font-size:12px;fill:#4b5563}",
            ".label{font-size:13px;font-weight:600}",
            ".title{font-size:20px;font-weight:700;fill:#111827}",
            "</style>",
            f'<rect width="{self.width}" height="{self.height}" fill="#ffffff"/>',
            f'<text x="{self.margin}" y="28" class="title">{escape(title)}</text>',
        ]

    def xy(self, x: float, y: float) -> tuple[float, float]:
        _, y_max, _ = self.scene.env
        return self.margin + x * self.scale, self.margin + (y_max - y) * self.scale

    def add(self, item: str):
        self.parts.append(item)

    def circle(self, zone: Zone, label_suffix: str = ""):
        cx, cy = self.xy(zone.x, zone.y)
        r = zone.r * self.scale
        label = f"{zone.label}{label_suffix}"
        self.add(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" '
            f'fill="{_hex_opacity(zone.color, zone.opacity)}" stroke="{zone.color}" '
            f'stroke-width="2.2"/>'
        )
        self.add(
            f'<text x="{cx + r + 6:.1f}" y="{cy - 4:.1f}" class="label">{escape(label)}</text>'
        )

    def polyline(self, traj: np.ndarray, color: str, width: float, dash: str | None = None):
        points = " ".join(f"{x:.1f},{y:.1f}" for x, y in (self.xy(float(p[0]), float(p[1])) for p in traj))
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-linecap="round" stroke-linejoin="round"{dash_attr}/>'
        )

    def marker(self, x: float, y: float, color: str, label: str, shape: str = "circle"):
        sx, sy = self.xy(x, y)
        if shape == "diamond":
            pts = [(sx, sy - 8), (sx + 8, sy), (sx, sy + 8), (sx - 8, sy)]
            point_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in pts)
            self.add(f'<polygon points="{point_str}" fill="{color}" stroke="#374151" stroke-width="1.5"/>')
        else:
            self.add(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="{color}" stroke="#374151" stroke-width="1.5"/>')
        self.add(f'<text x="{sx + 10:.1f}" y="{sy + 4:.1f}" class="label">{escape(label)}</text>')

    def finish(self) -> str:
        self.parts.append("</svg>")
        return "\n".join(self.parts)


def _draw_grid(canvas: SvgCanvas):
    scene = canvas.scene
    x_max, y_max, _ = scene.env
    mx, my = canvas.xy(0, 0)
    mx2, my2 = canvas.xy(x_max, y_max)
    left = min(mx, mx2)
    right = max(mx, mx2)
    top = min(my, my2)
    bottom = max(my, my2)
    canvas.add(f'<rect x="{left:.1f}" y="{top:.1f}" width="{right-left:.1f}" height="{bottom-top:.1f}" fill="#f8fafc" stroke="#7c3aed" stroke-width="2.5"/>')

    for y in np.linspace(y_max * 0.18, y_max * 0.82, 4):
        x1, yy = canvas.xy(0, y)
        x2, _ = canvas.xy(x_max, y)
        canvas.add(f'<line x1="{x1:.1f}" y1="{yy:.1f}" x2="{x2:.1f}" y2="{yy:.1f}" stroke="rgba(71,85,105,0.16)" stroke-width="2"/>')
    for x in np.linspace(x_max * 0.18, x_max * 0.82, 4):
        xx, y1 = canvas.xy(x, 0)
        _, y2 = canvas.xy(x, y_max)
        canvas.add(f'<line x1="{xx:.1f}" y1="{y1:.1f}" x2="{xx:.1f}" y2="{y2:.1f}" stroke="rgba(71,85,105,0.16)" stroke-width="2"/>')


def _draw_legend(canvas: SvgCanvas, rules: list[Rule], opt_source: str, crowd_buffer: float = 0.0):
    x = canvas.margin + canvas.map_w + 36
    y = canvas.margin + 22
    panel_x = x - 18
    panel_y = y - 30
    panel_h = canvas.height - panel_y - canvas.margin
    canvas.add(
        f'<rect x="{panel_x:.1f}" y="{panel_y:.1f}" width="{canvas.legend_w - 36:.1f}" '
        f'height="{panel_h:.1f}" rx="10" fill="#f8fafc" stroke="#d1d5db" stroke-width="1.2"/>'
    )
    canvas.add(f'<text x="{x:.1f}" y="{y:.1f}" class="small">Ground-plane view (metres)</text>')
    y += 30
    canvas.add(f'<text x="{x:.1f}" y="{y:.1f}" class="label">Trajectories</text>')
    y += 24
    canvas.add(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{x+46:.1f}" y2="{y:.1f}" stroke="#ff5c5c" stroke-width="4" stroke-dasharray="10 7"/>')
    canvas.add(f'<text x="{x+58:.1f}" y="{y+4:.1f}" class="small">Naive / rule-unaware route</text>')
    y += 24
    canvas.add(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{x+46:.1f}" y2="{y:.1f}" stroke="#2ecc71" stroke-width="5"/>')
    canvas.add(f'<text x="{x+58:.1f}" y="{y+4:.1f}" class="small">Compliant route ({escape(opt_source)})</text>')

    y += 36
    canvas.add(f'<text x="{x:.1f}" y="{y:.1f}" class="label">Map symbols</text>')
    y += 22
    canvas.add(f'<circle cx="{x+7:.1f}" cy="{y-4:.1f}" r="7" fill="#ffffff" stroke="#374151" stroke-width="1.5"/>')
    canvas.add(f'<text x="{x+24:.1f}" y="{y:.1f}" class="small">Home / launch point</text>')
    y += 22
    diamond = [(x + 7, y - 12), (x + 15, y - 4), (x + 7, y + 4), (x - 1, y - 4)]
    canvas.add(f'<polygon points="{" ".join(f"{px:.1f},{py:.1f}" for px, py in diamond)}" fill="#f1c40f" stroke="#374151" stroke-width="1.5"/>')
    canvas.add(f'<text x="{x+24:.1f}" y="{y:.1f}" class="small">Delivery / mission target</text>')
    y += 22
    canvas.add(f'<line x1="{x:.1f}" y1="{y-4:.1f}" x2="{x+16:.1f}" y2="{y-4:.1f}" stroke="#7c3aed" stroke-width="2.5" stroke-dasharray="3 3"/>')
    canvas.add(f'<text x="{x+24:.1f}" y="{y:.1f}" class="small">Operational-volume boundary</text>')

    zone_entries = [
        (canvas.scene.crowd_zones, "Crowd standoff area"),
        (canvas.scene.no_fly_zones, "No-fly area"),
        (canvas.scene.quiet_zones, "Quiet / noise-limited area"),
        (canvas.scene.emergency_zones, "Emergency area (activation shown on map)"),
    ]
    if canvas.scene.safe_landing is not None:
        zone_entries.append(([Zone("", 0, 0, 0, 0, "#2ecc71", 0.18)], "Safe landing area"))
    for zones, label in zone_entries:
        if not zones:
            continue
        y += 22
        zone = zones[0]
        canvas.add(f'<circle cx="{x+7:.1f}" cy="{y-4:.1f}" r="7" fill="{_hex_opacity(zone.color, max(zone.opacity, 0.35))}" stroke="{zone.color}" stroke-width="2"/>')
        canvas.add(f'<text x="{x+24:.1f}" y="{y:.1f}" class="small">{escape(label)}</text>')

    if crowd_buffer > 0.0:
        y += 22
        canvas.add(f'<circle cx="{x+7:.1f}" cy="{y-4:.1f}" r="7" fill="none" stroke="#c0392b" stroke-width="2" stroke-dasharray="4 3"/>')
        canvas.add(f'<text x="{x+24:.1f}" y="{y:.1f}" class="small">Additional crowd buffer: {crowd_buffer:g} m</text>')

    y += 42
    canvas.add(f'<text x="{x:.1f}" y="{y:.1f}" class="label">Scenario rules and colors</text>')
    y += 22
    for rule in sorted(rules, key=lambda r: (r.tier, r.id)):
        canvas.add(f'<rect x="{x:.1f}" y="{y-10:.1f}" width="14" height="14" fill="{rule.color}"/>')
        lines = RULE_PLOT_NAMES.get(rule.id, (rule.label,))
        tspans = [
            f'<tspan x="{x+24:.1f}" dy="{0 if index == 0 else 16:.1f}">{escape(line)}</tspan>'
            for index, line in enumerate(lines)
        ]
        canvas.add(
            f'<text x="{x+24:.1f}" y="{y+2:.1f}" class="small">'
            f'<tspan x="{x+24:.1f}" dy="0">{escape(lines[0])}</tspan>'
            f'{"".join(tspans[1:])}</text>'
        )
        y += 20 + 16 * (len(lines) - 1)


def save_2d_plot(
    title: str,
    scene: Scene,
    rules: list[Rule],
    traj_naive: np.ndarray,
    traj_opt: np.ndarray,
    opt_source: str,
    out_path: Path,
    crowd_buffer: float = 0.0,
) -> Path:
    canvas = SvgCanvas(scene, title)
    _draw_grid(canvas)

    for zone in scene.quiet_zones:
        suffix = f" <= {zone.noise_limit:.0f} dB" if zone.noise_limit is not None else ""
        canvas.circle(zone, suffix)
    for zone in scene.crowd_zones:
        if crowd_buffer > 0.0:
            cx, cy = canvas.xy(zone.x, zone.y)
            radius = (zone.r + crowd_buffer) * canvas.scale
            canvas.add(
                f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{radius:.1f}" fill="none" '
                f'stroke="{zone.color}" stroke-width="2" stroke-dasharray="8 6"/>'
            )
        canvas.circle(zone)
    for zone in scene.no_fly_zones:
        canvas.circle(zone)
    for zone in scene.emergency_zones:
        suffix = f" active t>={zone.active_from:.0f}s" if zone.active_from is not None else ""
        canvas.circle(zone, suffix)

    if scene.safe_landing is not None:
        sx, sy, _ = scene.safe_landing
        safe_zone = Zone("Safe landing", sx, sy, scene.landing_radius, 0.0, "#2ecc71", 0.18)
        canvas.circle(safe_zone)

    canvas.polyline(traj_naive, "#ff5c5c", 4.0, dash="10 7")
    canvas.polyline(traj_opt, "#2ecc71", 5.0)

    canvas.marker(scene.start[0], scene.start[1], "#ffffff", "Home")
    canvas.marker(scene.delivery[0], scene.delivery[1], "#f1c40f", "Delivery", shape="diamond")
    if scene.safe_landing is not None:
        sx, sy, _ = scene.safe_landing
        canvas.marker(sx, sy, "#2ecc71", "Safe landing")

    _draw_legend(canvas, rules, opt_source, crowd_buffer)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(canvas.finish(), encoding="utf-8")
    return out_path
