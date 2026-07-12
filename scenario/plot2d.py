from __future__ import annotations

from html import escape
from pathlib import Path

import numpy as np

from .core import Rule
from .scene import Scene, Zone


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
        self.legend_w = 300
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
            "text{font-family:Inter,Arial,sans-serif;fill:#d6dee8}",
            ".small{font-size:12px;fill:#aeb8c5}",
            ".label{font-size:13px;font-weight:600}",
            ".title{font-size:20px;font-weight:700;fill:#f5f7fb}",
            "</style>",
            f'<rect width="{self.width}" height="{self.height}" fill="#0d1117"/>',
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

    def polyline(self, traj: np.ndarray, color: str, width: float, label: str, dash: str | None = None):
        points = " ".join(f"{x:.1f},{y:.1f}" for x, y in (self.xy(float(p[0]), float(p[1])) for p in traj))
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="{width}" '
            f'stroke-linecap="round" stroke-linejoin="round"{dash_attr}/>'
        )
        mid = traj[len(traj) // 2]
        mx, my = self.xy(float(mid[0]), float(mid[1]))
        self.add(f'<text x="{mx + 8:.1f}" y="{my - 8:.1f}" class="label">{escape(label)}</text>')

    def marker(self, x: float, y: float, color: str, label: str, shape: str = "circle"):
        sx, sy = self.xy(x, y)
        if shape == "diamond":
            pts = [(sx, sy - 8), (sx + 8, sy), (sx, sy + 8), (sx - 8, sy)]
            point_str = " ".join(f"{px:.1f},{py:.1f}" for px, py in pts)
            self.add(f'<polygon points="{point_str}" fill="{color}" stroke="#ffffff" stroke-width="1.5"/>')
        else:
            self.add(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="{color}" stroke="#ffffff" stroke-width="1.5"/>')
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
    canvas.add(f'<rect x="{left:.1f}" y="{top:.1f}" width="{right-left:.1f}" height="{bottom-top:.1f}" fill="#151b23" stroke="#9b59b6" stroke-width="2.5"/>')

    for y in np.linspace(y_max * 0.18, y_max * 0.82, 4):
        x1, yy = canvas.xy(0, y)
        x2, _ = canvas.xy(x_max, y)
        canvas.add(f'<line x1="{x1:.1f}" y1="{yy:.1f}" x2="{x2:.1f}" y2="{yy:.1f}" stroke="rgba(255,255,255,0.14)" stroke-width="2"/>')
    for x in np.linspace(x_max * 0.18, x_max * 0.82, 4):
        xx, y1 = canvas.xy(x, 0)
        _, y2 = canvas.xy(x, y_max)
        canvas.add(f'<line x1="{xx:.1f}" y1="{y1:.1f}" x2="{xx:.1f}" y2="{y2:.1f}" stroke="rgba(255,255,255,0.14)" stroke-width="2"/>')


def _draw_legend(canvas: SvgCanvas, rules: list[Rule], opt_source: str):
    x = canvas.margin + canvas.map_w + 36
    y = canvas.margin + 22
    canvas.add(f'<text x="{x:.1f}" y="{y:.1f}" class="label">Trajectories</text>')
    y += 24
    canvas.add(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{x+46:.1f}" y2="{y:.1f}" stroke="#ff5c5c" stroke-width="4" stroke-dasharray="10 7"/>')
    canvas.add(f'<text x="{x+58:.1f}" y="{y+4:.1f}" class="small">naive</text>')
    y += 24
    canvas.add(f'<line x1="{x:.1f}" y1="{y:.1f}" x2="{x+46:.1f}" y2="{y:.1f}" stroke="#2ecc71" stroke-width="5"/>')
    canvas.add(f'<text x="{x+58:.1f}" y="{y+4:.1f}" class="small">{escape(opt_source)}</text>')

    y += 42
    canvas.add(f'<text x="{x:.1f}" y="{y:.1f}" class="label">Rule priority colors</text>')
    y += 22
    for rule in sorted(rules, key=lambda r: (r.tier, r.id)):
        canvas.add(f'<rect x="{x:.1f}" y="{y-10:.1f}" width="14" height="14" fill="{rule.color}"/>')
        canvas.add(f'<text x="{x+24:.1f}" y="{y+2:.1f}" class="small">T{rule.tier} {escape(rule.id)} - {escape(rule.label)}</text>')
        y += 22


def save_2d_plot(
    title: str,
    scene: Scene,
    rules: list[Rule],
    traj_naive: np.ndarray,
    traj_opt: np.ndarray,
    opt_source: str,
    out_path: Path,
) -> Path:
    canvas = SvgCanvas(scene, title)
    _draw_grid(canvas)

    for zone in scene.quiet_zones:
        suffix = f" <= {zone.noise_limit:.0f} dB" if zone.noise_limit is not None else ""
        canvas.circle(zone, suffix)
    for zone in scene.crowd_zones:
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

    canvas.polyline(traj_naive, "#ff5c5c", 4.0, "naive", dash="10 7")
    canvas.polyline(traj_opt, "#2ecc71", 5.0, "compliant")

    canvas.marker(scene.start[0], scene.start[1], "#ffffff", "Home")
    canvas.marker(scene.delivery[0], scene.delivery[1], "#f1c40f", "Delivery", shape="diamond")
    if scene.safe_landing is not None:
        sx, sy, _ = scene.safe_landing
        canvas.marker(sx, sy, "#2ecc71", "Safe landing")

    _draw_legend(canvas, rules, opt_source)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(canvas.finish(), encoding="utf-8")
    return out_path
