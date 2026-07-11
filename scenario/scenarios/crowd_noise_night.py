"""Crowd/noise/night scenario.

The map contains crowd standoff zones, a residential quiet zone, and a mission
that crosses from day into night. The naive path cuts through people/noise
constraints and keeps lights off; the compliant path reroutes and enables night
lighting.
"""

from __future__ import annotations

from scenario.core import Rule, ScenarioBase
from scenario.scene import Scene, Zone


class CrowdNoiseNightScenario(ScenarioBase):
    key = "crowd_noise_night"
    title = "Scenario 1 - Crowd, Quiet Zone, Night Lighting"
    night_start = 95.0
    mission_time = 180.0
    noise_base = 50.0
    noise_slope = 1.8
    n_free = 4
    penalty_weight = 20.0

    def build_rules(self) -> list[Rule]:
        return [
            Rule("REQ-PROX-01", 1, "Crowd standoff", "Crowd", "#c0392b", "crowd"),
            Rule("REQ-ZONE-03", 3, "Altitude ceiling", "Alt", "#e74c3c", "altitude"),
            Rule("REQ-EQP-07", 5, "Maximum speed", "Speed", "#9b59b6", "speed"),
            Rule("REQ-EQP-04", 6, "Night light", "Light", "#f1c40f", "night_light"),
            Rule("REQ-EQP-12", 7, "Noise in quiet zone", "Noise", "#3498db", "noise_zone"),
        ]

    def build_scene(self) -> Scene:
        return Scene(
            env=(1000.0, 700.0, 160.0),
            start=(80.0, 90.0, 35.0),
            delivery=(910.0, 610.0, 35.0),
            naive_wps=[
                (80.0, 90.0, 35.0),
                (420.0, 340.0, 55.0),
                (690.0, 360.0, 60.0),
                (910.0, 610.0, 35.0),
            ],
            fallback_wps=[
                (80.0, 90.0, 35.0),
                (225.0, 115.0, 55.0),
                (285.0, 620.0, 58.0),
                (760.0, 655.0, 55.0),
                (910.0, 610.0, 35.0),
            ],
            crowd_zones=[
                Zone("Assembly of people", 420.0, 340.0, 105.0, 70.0, "#c0392b", 0.30),
                Zone("Pedestrian cluster", 590.0, 465.0, 62.0, 45.0, "#d35400", 0.22),
            ],
            quiet_zones=[
                Zone("Residential quiet zone", 690.0, 360.0, 165.0, 95.0, "#3498db", 0.20, noise_limit=58.0),
            ],
        )
