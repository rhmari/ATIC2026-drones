from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import plotly.graph_objects as go

from .scene import Scene, add_scene_traces

try:
    from scipy.optimize import minimize as sp_minimize

    SCIPY_OK = True
except ImportError:
    SCIPY_OK = False


C_OK = "#2ecc71"
FRAME_STEP = 4
FRAME_DUR = 45
TRAIL_LEN = 56


@dataclass(frozen=True)
class Rule:
    id: str
    tier: int
    label: str
    short: str
    color: str
    kind: str


@dataclass(frozen=True)
class Policy:
    light_after_night: bool = False


class ScenarioBase:
    key = "base"
    title = "Base scenario"
    mission_time = 180.0
    altitude_max = 120.0
    speed_max = 19.0
    noise_base = 50.0
    noise_slope = 1.8
    night_start: float | None = None
    battery_start = 100.0
    battery_end = 70.0
    battery_warn = 20.0
    n_free = 4
    t_points = 360
    t_opt = 90
    penalty_weight = 18.0
    robustness_margin = 1e-3
    constraint_tolerance = 1e-6
    rule_margins: dict[str, float] = {}

    def __init__(self):
        self.scene = self.build_scene()
        self.rules = self.build_rules()

    def build_scene(self) -> Scene:
        raise NotImplementedError

    def build_rules(self) -> list[Rule]:
        raise NotImplementedError

    def catmull_rom(self, waypoints: list[tuple[float, float, float]], n_points: int) -> np.ndarray:
        pts = np.array(waypoints, dtype=float)
        if len(pts) == 1:
            return np.repeat(pts, n_points, axis=0)

        segs = len(pts) - 1
        pps = max(2, n_points // segs)
        out = []
        for i in range(segs):
            p0 = pts[max(i - 1, 0)]
            p1 = pts[i]
            p2 = pts[i + 1]
            p3 = pts[min(i + 2, len(pts) - 1)]
            count = pps if i < segs - 1 else n_points - len(out)
            for t in np.linspace(0.0, 1.0, count, endpoint=(i == segs - 1)):
                t2 = t * t
                t3 = t2 * t
                out.append(
                    0.5
                    * (
                        2 * p1
                        + (-p0 + p2) * t
                        + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                        + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
                    )
                )
        return np.array(out)

    def trajectory_from_waypoints(
        self,
        waypoints: list[tuple[float, float, float]],
        n_points: int,
        hold_fraction: float = 1.0,
    ) -> np.ndarray:
        hold_fraction = float(np.clip(hold_fraction, 0.05, 1.0))
        if hold_fraction >= 0.999:
            return self.catmull_rom(waypoints, n_points)

        moving_n = max(4, int(round(n_points * hold_fraction)))
        moving = self.catmull_rom(waypoints, moving_n)
        hold = np.repeat(moving[-1:, :], n_points - moving_n, axis=0)
        return np.vstack([moving, hold])

    def make_trajectory_from_free(self, params: np.ndarray, n_points: int) -> np.ndarray:
        free = params.reshape(self.n_free, 3)
        goal = self.scene.opt_goal if self.scene.opt_goal is not None else self.scene.delivery
        wps = [self.scene.start] + [tuple(p) for p in free] + [goal]
        return self.trajectory_from_waypoints(wps, n_points, self.scene.opt_hold_fraction)

    def make_initial_params(self) -> np.ndarray:
        # Prefer an exact representation of the verified fallback whenever its
        # number of intermediate waypoints matches the optimisation model.
        # This gives the constrained solver a strictly feasible initial point.
        if len(self.scene.fallback_wps) == self.n_free + 2:
            pts = np.array(self.scene.fallback_wps[1:-1], dtype=float)
        else:
            seed = self.catmull_rom(self.scene.fallback_wps, self.n_free + 2)
            pts = seed[1:-1].copy()
        pts[:, 2] = np.clip(pts[:, 2], 8.0, self.altitude_max - 2.0)
        return pts.flatten()

    def compute_signals(self, traj: np.ndarray, policy: Policy) -> dict[str, np.ndarray]:
        n = len(traj)
        dt = self.mission_time / max(1, n - 1)
        dp = np.diff(traj, axis=0, prepend=traj[[0]])
        speed = np.linalg.norm(dp, axis=1) / dt
        if n > 1:
            speed[0] = speed[1]

        time = np.linspace(0.0, self.mission_time, n)
        battery = np.linspace(self.battery_start, self.battery_end, n)
        noise = self.noise_base + self.noise_slope * speed
        night = np.zeros(n, dtype=bool)
        if self.night_start is not None:
            night = time >= self.night_start
        light_on = night & policy.light_after_night

        return dict(time=time, speed=speed, battery=battery, noise=noise, night=night, light_on=light_on)

    @staticmethod
    def _circle_distance(traj: np.ndarray, zone) -> np.ndarray:
        return np.sqrt((traj[:, 0] - zone.x) ** 2 + (traj[:, 1] - zone.y) ** 2)

    @staticmethod
    def _min_or_inf(values: list[np.ndarray], n: int) -> np.ndarray:
        if not values:
            return np.full(n, np.inf)
        out = np.full(n, np.inf)
        for v in values:
            out = np.minimum(out, v)
        return out

    def robustness(self, rule: Rule, traj: np.ndarray, sig: dict[str, np.ndarray]) -> np.ndarray:
        scene = self.scene
        n = len(traj)
        x = traj[:, 0]
        y = traj[:, 1]
        z = traj[:, 2]

        if rule.kind == "crowd":
            return self._min_or_inf([self._circle_distance(traj, zn) - zn.r for zn in scene.crowd_zones], n)

        if rule.kind == "geofence":
            x_max, y_max, _ = scene.env
            return np.minimum(np.minimum(x, x_max - x), np.minimum(y, y_max - y))

        if rule.kind == "altitude":
            return self.altitude_max - z

        if rule.kind == "no_fly":
            vals = []
            for zn in scene.no_fly_zones:
                dist = self._circle_distance(traj, zn)
                vals.append(np.where(z <= zn.h, dist - zn.r, np.inf))
            return self._min_or_inf(vals, n)

        if rule.kind == "emergency":
            vals = []
            for zn in scene.emergency_zones:
                dist = self._circle_distance(traj, zn)
                active = sig["time"] >= float(zn.active_from or 0.0)
                vals.append(np.where(active & (z <= zn.h), dist - zn.r, np.inf))
            return self._min_or_inf(vals, n)

        if rule.kind == "noise_zone":
            vals = []
            for zn in scene.quiet_zones:
                dist = self._circle_distance(traj, zn)
                inside_rob = zn.r - dist
                noise_limit = float(zn.noise_limit if zn.noise_limit is not None else 70.0)
                noise_rob = noise_limit - sig["noise"]
                vals.append(np.maximum(-inside_rob, noise_rob))
            return self._min_or_inf(vals, n)

        if rule.kind == "night_light":
            return np.where(sig["night"], np.where(sig["light_on"], 1.0, -1.0), 1.0)

        if rule.kind == "speed":
            return self.speed_max - sig["speed"]

        if rule.kind == "battery_landing":
            if scene.safe_landing is None:
                return sig["battery"] - self.battery_warn
            sx, sy, sz = scene.safe_landing
            dist = np.sqrt((x - sx) ** 2 + (y - sy) ** 2)
            landing_rob = np.minimum(scene.landing_radius - dist, scene.landing_altitude - np.abs(z - sz))
            return np.maximum(sig["battery"] - self.battery_warn, landing_rob)

        raise ValueError(f"Unknown rule kind: {rule.kind}")

    def monitor_for(self, traj: np.ndarray, policy: Policy):
        sig = self.compute_signals(traj, policy)
        results = []
        for rule in self.rules:
            rob = self.robustness(rule, traj, sig)
            results.append(dict(rob=rob, ok=(rob >= 0.0)))

        all_ok = np.ones(len(traj), dtype=bool)
        for rr in results:
            all_ok &= rr["ok"]

        order = sorted(range(len(self.rules)), key=lambda idx: self.rules[idx].tier)
        colors = []
        for i in range(len(traj)):
            color = C_OK
            for idx in order:
                if not results[idx]["ok"][i]:
                    color = self.rules[idx].color
                    break
            colors.append(color)

        return results, all_ok, colors

    def violation_depth(self, traj: np.ndarray, policy: Policy) -> float:
        sig = self.compute_signals(traj, policy)
        total = 0.0
        for rule in self.rules:
            rob = self.robustness(rule, traj, sig)
            finite = rob[np.isfinite(rob)]
            if len(finite):
                total += float(np.sum(np.maximum(0.0, -finite)))
        return total

    def objective(self, params: np.ndarray, policy: Policy) -> float:
        traj = self.make_trajectory_from_free(params, self.t_opt)
        path_len = self.path_length(traj)
        return path_len + self.penalty_weight * self.violation_depth(traj, policy)

    @staticmethod
    def path_length(traj: np.ndarray) -> float:
        return float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))

    def sampled_robustness_constraints(self, params: np.ndarray, policy: Policy) -> np.ndarray:
        """Return monitor-grid STL robustness values for all selected rules.

        SLSQP treats every returned component as a separate inequality constraint.
        An inactive conditional rule has infinite robustness in the monitor; it is
        omitted because it imposes no constraint at that sample.
        """
        # Use the same 360-sample grid as the final monitor. Optimising on a
        # coarser grid can miss a spline incursion between constraint samples.
        traj = self.make_trajectory_from_free(params, self.t_points)
        sig = self.compute_signals(traj, policy)
        values = []
        for rule in self.rules:
            rob = self.robustness(rule, traj, sig)
            values.append(rob[np.isfinite(rob)])
        return np.concatenate(values) if values else np.array([1.0])

    def constraint_residuals(self, params: np.ndarray, policy: Policy) -> np.ndarray:
        """STL constraint residuals after applying per-rule safety margins."""
        traj = self.make_trajectory_from_free(params, self.t_points)
        sig = self.compute_signals(traj, policy)
        values = []
        for rule in self.rules:
            rob = self.robustness(rule, traj, sig)
            margin = self.rule_margins.get(rule.id, self.robustness_margin)
            values.append(rob[np.isfinite(rob)] - margin)
        return np.concatenate(values) if values else np.array([1.0])

    def hard_stl_objective(self, params: np.ndarray) -> float:
        return self.path_length(self.make_trajectory_from_free(params, self.t_opt))

    def optimized_trajectory(self, no_opt: bool = False) -> tuple[np.ndarray, str]:
        fallback = self.trajectory_from_waypoints(
            self.scene.fallback_wps,
            self.t_points,
            self.scene.opt_hold_fraction,
        )
        if no_opt or not SCIPY_OK:
            return fallback, "fallback compliant path"

        bounds = []
        x_max, y_max, _ = self.scene.env
        for _ in range(self.n_free):
            bounds.extend([(8.0, x_max - 8.0), (8.0, y_max - 8.0), (8.0, self.altitude_max - 2.0)])

        opt_policy = Policy(light_after_night=True)

        # Phase 1: restore sampled feasibility from the waypoint seed. The
        # fallback path provides the intended safe homotopy class, but its
        # resampled waypoint representation can initially violate speed limits.
        initial = self.make_initial_params()
        if np.min(self.constraint_residuals(initial, opt_policy)) >= 0.0:
            restore_x = initial
        else:
            restore = sp_minimize(
                lambda p: self.objective(p, opt_policy),
                initial,
                method="L-BFGS-B",
                bounds=bounds,
                options=dict(maxiter=1600, ftol=1e-9, gtol=1e-6),
            )
            restore_x = restore.x

        # Phase 2: minimise path length with all sampled rule constraints hard.
        constraints = {
            "type": "ineq",
            "fun": lambda p: self.constraint_residuals(p, opt_policy),
        }
        res = sp_minimize(
            self.hard_stl_objective,
            restore_x,
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            options=dict(maxiter=1600, ftol=1e-8, disp=False),
        )
        candidate = self.make_trajectory_from_free(res.x, self.t_points)
        full_depth = self.violation_depth(candidate, opt_policy)
        full_min = float(np.min(self.constraint_residuals(res.x, opt_policy)))

        if (
            res.success
            and full_depth <= self.constraint_tolerance
            and full_min >= -self.constraint_tolerance
        ):
            return candidate, f"sampled STL constraints ({res.nit} iterations)"
        return fallback, "fallback compliant path"

    def build_figure(
        self,
        traj_naive: np.ndarray,
        traj_opt: np.ndarray,
        naive_results,
        naive_all_ok,
        naive_colors,
        opt_results,
        opt_colors,
        opt_source: str,
    ) -> go.Figure:
        fig = go.Figure()
        add_scene_traces(fig, self.scene, self.altitude_max)

        rob_matrix_opt = np.column_stack([rr["rob"] for rr in opt_results])
        hover_opt = (
            "<b>COMPLIANT (%{x:.0f} m, %{y:.0f} m, %{z:.0f} m)</b><br>"
            + "".join(f'{self.rules[j].id}: %{{customdata[{j}]:+.1f}}<br>' for j in range(len(self.rules)))
            + "<extra></extra>"
        )
        fig.add_trace(
            go.Scatter3d(
                x=traj_opt[:, 0],
                y=traj_opt[:, 1],
                z=traj_opt[:, 2],
                mode="markers",
                marker=dict(size=3.0, color=opt_colors, opacity=0.92),
                name=f"Compliant path - {opt_source}",
                customdata=rob_matrix_opt,
                hovertemplate=hover_opt,
                showlegend=True,
            )
        )

        fig.add_trace(
            go.Scatter3d(
                x=traj_naive[:, 0],
                y=traj_naive[:, 1],
                z=traj_naive[:, 2],
                mode="lines",
                line=dict(color="rgba(255,255,255,0.13)", width=2),
                showlegend=False,
                hoverinfo="skip",
            )
        )

        fig.add_trace(
            go.Scatter3d(
                x=[None],
                y=[None],
                z=[None],
                mode="markers",
                marker=dict(size=9, color=C_OK),
                name="Compliant",
                showlegend=True,
            )
        )
        for rule in self.rules:
            fig.add_trace(
                go.Scatter3d(
                    x=[None],
                    y=[None],
                    z=[None],
                    mode="markers",
                    marker=dict(size=9, color=rule.color),
                    name=f"[T{rule.tier}] {rule.id}: {rule.label}",
                    showlegend=True,
                )
            )

        n_static = len(fig.data)
        rob_matrix_naive = np.column_stack([rr["rob"] for rr in naive_results])
        hover_naive = (
            "<b>NAIVE (%{x:.0f} m, %{y:.0f} m, %{z:.0f} m)</b><br>"
            + "".join(f'{self.rules[j].id}: %{{customdata[{j}]:+.1f}}<br>' for j in range(len(self.rules)))
            + "<extra></extra>"
        )

        fig.add_trace(
            go.Scatter3d(
                x=[traj_naive[0, 0]],
                y=[traj_naive[0, 1]],
                z=[traj_naive[0, 2]],
                mode="markers",
                marker=dict(size=3.5, color=[naive_colors[0]], opacity=0.95),
                customdata=rob_matrix_naive[[0]],
                hovertemplate=hover_naive,
                showlegend=False,
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=[traj_naive[0, 0]],
                y=[traj_naive[0, 1]],
                z=[traj_naive[0, 2]],
                mode="markers",
                marker=dict(size=14, color=naive_colors[0], symbol="circle", line=dict(color="white", width=2)),
                name="Naive drone",
                showlegend=True,
            )
        )

        def status(i: int) -> str:
            parts = [f"{r.short} {'OK' if naive_results[k]['ok'][i] else 'FAIL'}" for k, r in enumerate(self.rules)]
            pct = 100 * naive_all_ok[: i + 1].mean()
            return f"{self.title} - Naive step {i + 1}/{len(traj_naive)} | {' / '.join(parts)} | compliant {pct:.0f}%"

        frames = []
        for fi in range(0, len(traj_naive), FRAME_STEP):
            t0 = max(0, fi - TRAIL_LEN)
            sl = slice(t0, fi + 1)
            sizes = np.linspace(1.5, 4.5, fi + 1 - t0).tolist()
            frames.append(
                go.Frame(
                    name=str(fi),
                    traces=[n_static, n_static + 1],
                    data=[
                        go.Scatter3d(
                            x=traj_naive[sl, 0].tolist(),
                            y=traj_naive[sl, 1].tolist(),
                            z=traj_naive[sl, 2].tolist(),
                            mode="markers",
                            marker=dict(size=sizes, color=naive_colors[t0 : fi + 1], opacity=0.92),
                            customdata=rob_matrix_naive[sl],
                            hovertemplate=hover_naive,
                        ),
                        go.Scatter3d(
                            x=[traj_naive[fi, 0]],
                            y=[traj_naive[fi, 1]],
                            z=[traj_naive[fi, 2]],
                            mode="markers",
                            marker=dict(
                                size=14,
                                color=naive_colors[fi],
                                symbol="circle",
                                line=dict(color="white", width=2),
                            ),
                        ),
                    ],
                    layout=go.Layout(title=dict(text=status(fi))),
                )
            )
        fig.frames = frames

        x_max, y_max, z_max = self.scene.env
        fig.update_layout(
            title=dict(text=self.title, font=dict(size=14, color="white"), x=0.5, xanchor="center"),
            paper_bgcolor="#0d1117",
            scene=dict(
                xaxis=dict(
                    title="East (m)",
                    range=[0, x_max],
                    backgroundcolor="#161b22",
                    gridcolor="#30363d",
                    color="#8b949e",
                ),
                yaxis=dict(
                    title="North (m)",
                    range=[0, y_max],
                    backgroundcolor="#161b22",
                    gridcolor="#30363d",
                    color="#8b949e",
                ),
                zaxis=dict(
                    title="Altitude (m)",
                    range=[0, z_max],
                    backgroundcolor="#161b22",
                    gridcolor="#30363d",
                    color="#8b949e",
                ),
                bgcolor="#161b22",
                aspectmode="manual",
                aspectratio=dict(x=1, y=y_max / x_max, z=0.28),
                camera=dict(eye=dict(x=1.6, y=-1.6, z=1.0)),
            ),
            legend=dict(
                bgcolor="rgba(13,17,23,0.88)",
                bordercolor="#30363d",
                borderwidth=1,
                font=dict(color="white", size=10),
                x=0.01,
                y=0.99,
                xanchor="left",
                yanchor="top",
            ),
            updatemenus=[
                dict(
                    type="buttons",
                    showactive=False,
                    bgcolor="#161b22",
                    bordercolor="#30363d",
                    font=dict(color="white"),
                    x=0.5,
                    xanchor="center",
                    y=-0.08,
                    yanchor="top",
                    buttons=[
                        dict(
                            label="Play naive",
                            method="animate",
                            args=[
                                None,
                                dict(
                                    frame=dict(duration=FRAME_DUR, redraw=True),
                                    fromcurrent=True,
                                    transition=dict(duration=0),
                                    mode="immediate",
                                ),
                            ],
                        ),
                        dict(
                            label="Pause",
                            method="animate",
                            args=[
                                [None],
                                dict(frame=dict(duration=0, redraw=False), mode="immediate", transition=dict(duration=0)),
                            ],
                        ),
                    ],
                )
            ],
            sliders=[
                dict(
                    active=0,
                    x=0.05,
                    y=-0.02,
                    len=0.90,
                    bgcolor="#161b22",
                    bordercolor="#30363d",
                    tickcolor="#8b949e",
                    font=dict(color="#8b949e", size=10),
                    currentvalue=dict(prefix="Step: ", visible=True, xanchor="center", font=dict(color="white", size=12)),
                    transition=dict(duration=0),
                    steps=[
                        dict(
                            method="animate",
                            label=str(fi),
                            args=[
                                [str(fi)],
                                dict(frame=dict(duration=0, redraw=True), mode="immediate", transition=dict(duration=0)),
                            ],
                        )
                        for fi in range(0, len(traj_naive), FRAME_STEP)
                    ],
                )
            ],
            margin=dict(l=0, r=0, t=50, b=100),
        )
        return fig

    @staticmethod
    def fmt_rule(results, idx: int, total: int) -> str:
        rr = results[idx]
        finite = rr["rob"][np.isfinite(rr["rob"])]
        status = "PASS" if rr["ok"].all() else "FAIL"
        min_rob = finite.min() if len(finite) else float("inf")
        return f"{status} {rr['ok'].sum():>4}/{total:<4} min={min_rob:>+7.1f}"

    def print_report(self, naive_results, naive_all_ok, opt_results, opt_all_ok, opt_source: str):
        print("=" * 88)
        print(f"  {self.title}")
        print(f"  compliant path source: {opt_source}")
        print("=" * 88)
        print(f"  {'Rule':<16}  {'Tier':<5}  {'Label':<30}  {'NAIVE':<24}  {'COMPLIANT':<24}")
        print("-" * 88)
        for idx, rule in enumerate(self.rules):
            print(
                f"  {rule.id:<16}  {rule.tier:<5}  {rule.label:<30}  "
                f"{self.fmt_rule(naive_results, idx, self.t_points):<24}  "
                f"{self.fmt_rule(opt_results, idx, self.t_points):<24}"
            )
        print("-" * 88)
        print(f"  Overall naive:     {naive_all_ok.sum():>4}/{self.t_points} ({100 * naive_all_ok.mean():.1f}%)")
        print(f"  Overall compliant: {opt_all_ok.sum():>4}/{self.t_points} ({100 * opt_all_ok.mean():.1f}%)")
        print("=" * 88 + "\n")

    def run(self, out_dir: Path, show: bool = False, no_opt: bool = False, write_html: bool = True):
        print(f"\nRunning {self.key} ...")
        if not SCIPY_OK:
            print("  scipy not found: using the handcrafted compliant fallback path.")

        naive_policy = Policy(light_after_night=False)
        opt_policy = Policy(light_after_night=True)

        traj_naive = self.trajectory_from_waypoints(self.scene.naive_wps, self.t_points)
        traj_opt, opt_source = self.optimized_trajectory(no_opt=no_opt)

        naive_results, naive_all_ok, naive_colors = self.monitor_for(traj_naive, naive_policy)
        opt_results, opt_all_ok, opt_colors = self.monitor_for(traj_opt, opt_policy)

        self.print_report(naive_results, naive_all_ok, opt_results, opt_all_ok, opt_source)

        from .plot2d import save_2d_plot

        plot2d_path = out_dir.parent / "output_2d" / f"{self.key}.svg"
        save_2d_plot(
            self.title,
            self.scene,
            self.rules,
            traj_naive,
            traj_opt,
            opt_source,
            plot2d_path,
            crowd_buffer=self.rule_margins.get("REQ-PROX-01", 0.0),
        )
        print(f"  wrote {plot2d_path}")

        if write_html:
            fig = self.build_figure(
                traj_naive,
                traj_opt,
                naive_results,
                naive_all_ok,
                naive_colors,
                opt_results,
                opt_colors,
                opt_source,
            )

            out_dir.mkdir(parents=True, exist_ok=True)
            out_path = out_dir / f"{self.key}.html"
            fig.write_html(str(out_path), include_plotlyjs=True)
            print(f"  wrote {out_path}")
            if show:
                fig.show()
