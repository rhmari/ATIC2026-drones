from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import plotly.graph_objects as go


@dataclass(frozen=True)
class Zone:
    label: str
    x: float
    y: float
    r: float
    h: float
    color: str
    opacity: float
    active_from: float | None = None
    noise_limit: float | None = None


@dataclass(frozen=True)
class Scene:
    env: tuple[float, float, float]
    start: tuple[float, float, float]
    delivery: tuple[float, float, float]
    naive_wps: list[tuple[float, float, float]]
    fallback_wps: list[tuple[float, float, float]]
    opt_goal: tuple[float, float, float] | None = None
    opt_hold_fraction: float = 1.0
    crowd_zones: list[Zone] = field(default_factory=list)
    quiet_zones: list[Zone] = field(default_factory=list)
    no_fly_zones: list[Zone] = field(default_factory=list)
    emergency_zones: list[Zone] = field(default_factory=list)
    safe_landing: tuple[float, float, float] | None = None
    landing_radius: float = 45.0
    landing_altitude: float = 5.0


def cylinder(zone: Zone, name: str | None = None, n: int = 72):
    th = np.linspace(0, 2 * np.pi, n)
    th_grid, z_grid = np.meshgrid(th, [0.0, zone.h])
    return go.Surface(
        x=zone.x + zone.r * np.cos(th_grid),
        y=zone.y + zone.r * np.sin(th_grid),
        z=z_grid,
        colorscale=[[0, zone.color], [1, zone.color]],
        showscale=False,
        opacity=zone.opacity,
        name=name or zone.label,
        showlegend=True,
        hovertemplate=f"<b>{zone.label}</b><extra></extra>",
    )


def disc(x: float, y: float, r: float, z: float, color: str, name: str, opacity: float = 0.65, n: int = 72):
    rv = np.linspace(0, r, 14)
    th = np.linspace(0, 2 * np.pi, n)
    rr, tt = np.meshgrid(rv, th)
    return go.Surface(
        x=x + rr * np.cos(tt),
        y=y + rr * np.sin(tt),
        z=np.full_like(rr, z),
        colorscale=[[0, color], [1, color]],
        showscale=False,
        opacity=opacity,
        name=name,
        showlegend=True,
        hovertemplate=f"<b>{name}</b><extra></extra>",
    )


def altitude_ceiling(scene: Scene, altitude_max: float):
    x_max, y_max, _ = scene.env
    xx = np.array([[0, x_max], [0, x_max]], dtype=float)
    yy = np.array([[0, 0], [y_max, y_max]], dtype=float)
    return go.Surface(
        x=xx,
        y=yy,
        z=np.full_like(xx, altitude_max),
        colorscale=[[0, "#2980b9"], [1, "#2980b9"]],
        showscale=False,
        opacity=0.13,
        name=f"Altitude ceiling ({int(altitude_max)} m)",
        showlegend=True,
    )


def geofence_edges(scene: Scene):
    x_max, y_max, z_max = scene.env
    corners = [(0, 0), (x_max, 0), (x_max, y_max), (0, y_max), (0, 0)]
    traces = []
    for z in (0.0, z_max):
        traces.append(
            go.Scatter3d(
                x=[c[0] for c in corners],
                y=[c[1] for c in corners],
                z=[z] * len(corners),
                mode="lines",
                line=dict(color="#9b59b6", width=4, dash="dot"),
                name="Operational volume" if z == 0 else "",
                showlegend=(z == 0),
            )
        )
    for px, py in corners[:-1]:
        traces.append(
            go.Scatter3d(
                x=[px, px],
                y=[py, py],
                z=[0.0, z_max],
                mode="lines",
                line=dict(color="#9b59b6", width=3, dash="dot"),
                showlegend=False,
                hoverinfo="skip",
            )
        )
    return traces


def add_map_decoration(fig: go.Figure, scene: Scene):
    x_max, y_max, _ = scene.env
    road_color = "rgba(255,255,255,0.16)"
    for y in np.linspace(y_max * 0.18, y_max * 0.82, 4):
        fig.add_trace(
            go.Scatter3d(
                x=[0, x_max],
                y=[y, y],
                z=[0.4, 0.4],
                mode="lines",
                line=dict(color=road_color, width=2),
                showlegend=False,
                hoverinfo="skip",
            )
        )
    for x in np.linspace(x_max * 0.18, x_max * 0.82, 4):
        fig.add_trace(
            go.Scatter3d(
                x=[x, x],
                y=[0, y_max],
                z=[0.4, 0.4],
                mode="lines",
                line=dict(color=road_color, width=2),
                showlegend=False,
                hoverinfo="skip",
            )
        )


def add_scene_traces(fig: go.Figure, scene: Scene, altitude_max: float):
    add_map_decoration(fig, scene)

    for tr in geofence_edges(scene):
        fig.add_trace(tr)
    fig.add_trace(altitude_ceiling(scene, altitude_max))

    for zn in scene.no_fly_zones:
        fig.add_trace(cylinder(zn))
        fig.add_trace(disc(zn.x, zn.y, zn.r, zn.h, zn.color, f"{zn.label} top", 0.18))
    for zn in scene.crowd_zones:
        fig.add_trace(cylinder(zn))
    for zn in scene.quiet_zones:
        fig.add_trace(cylinder(zn))
    for zn in scene.emergency_zones:
        label = f"{zn.label} (active after {int(zn.active_from or 0)} s)"
        fig.add_trace(cylinder(zn, name=label))

    fig.add_trace(disc(scene.delivery[0], scene.delivery[1], 32, 0.0, "#f1c40f", "Delivery target"))
    fig.add_trace(
        go.Scatter3d(
            x=[scene.start[0]],
            y=[scene.start[1]],
            z=[scene.start[2]],
            mode="markers+text",
            text=["  Home"],
            textfont=dict(color="white", size=12),
            marker=dict(size=10, color="white", symbol="circle"),
            name="Home",
        )
    )
    fig.add_trace(
        go.Scatter3d(
            x=[scene.delivery[0]],
            y=[scene.delivery[1]],
            z=[scene.delivery[2]],
            mode="markers+text",
            text=["  Delivery"],
            textfont=dict(color="#f1c40f", size=12),
            marker=dict(size=12, color="#f1c40f", symbol="diamond"),
            name="Delivery",
        )
    )

    if scene.safe_landing is not None:
        sx, sy, sz = scene.safe_landing
        fig.add_trace(disc(sx, sy, scene.landing_radius, sz, "#2ecc71", "Safe landing area", 0.55))
        fig.add_trace(
            go.Scatter3d(
                x=[sx],
                y=[sy],
                z=[max(8.0, sz + 8.0)],
                mode="markers+text",
                text=["  Safe landing"],
                textfont=dict(color="#2ecc71", size=12),
                marker=dict(size=10, color="#2ecc71", symbol="circle"),
                name="Safe landing point",
            )
        )
