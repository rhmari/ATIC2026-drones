"""Battery abort scenario.

The map places a restricted airspace zone between home and delivery, plus a safe
landing site. The compliant behavior is not to finish the delivery at all: it
avoids the restricted zone and lands once the battery rule becomes active.
"""

from __future__ import annotations

from scenario.core import Rule, ScenarioBase
from scenario.scene import Scene, Zone


class BatteryAbortScenario(ScenarioBase):
    key = "battery_abort"
    title = "Scenario 3 - Restricted Zone and Battery-Driven Safe Landing"
    mission_time = 220.0
    battery_start = 45.0
    battery_end = 8.0
    battery_warn = 20.0
    n_free = 3
    penalty_weight = 26.0

    def build_rules(self) -> list[Rule]:
        return [
            Rule("REQ-PROX-06", 2, "Operational volume", "Geo", "#9b59b6", "geofence"),
            Rule("REQ-ZONE-04", 3, "No-fly zone", "NFZ", "#e67e22", "no_fly"),
            Rule("REQ-ZONE-03", 3, "Altitude ceiling", "Alt", "#e74c3c", "altitude"),
            Rule("REQ-EQP-03", 5, "Low battery landing", "Battery", "#1abc9c", "battery_landing"),
            Rule("REQ-EQP-07", 5, "Maximum speed", "Speed", "#8e44ad", "speed"),
        ]

    def build_scene(self) -> Scene:
        return Scene(
            env=(1000.0, 650.0, 150.0),
            start=(90.0, 320.0, 35.0),
            delivery=(930.0, 320.0, 35.0),
            opt_goal=(510.0, 565.0, 0.0),
            opt_hold_fraction=0.58,
            naive_wps=[
                (90.0, 320.0, 35.0),
                (500.0, 320.0, 80.0),
                (930.0, 320.0, 35.0),
            ],
            fallback_wps=[
                (90.0, 320.0, 35.0),
                (230.0, 500.0, 45.0),
                (410.0, 585.0, 22.0),
                (510.0, 565.0, 0.0),
            ],
            no_fly_zones=[
                Zone("Restricted airspace", 500.0, 320.0, 125.0, 140.0, "#e67e22", 0.34),
            ],
            safe_landing=(510.0, 565.0, 0.0),
            landing_radius=52.0,
            landing_altitude=7.0,
        )
