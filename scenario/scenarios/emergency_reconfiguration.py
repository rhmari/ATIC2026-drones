"""Emergency reconfiguration scenario.

The map combines a static hospital helipad no-fly zone with a dynamic emergency
response area that becomes active mid-flight. The scenario highlights priority:
human/emergency exclusion dominates route efficiency and altitude shortcuts.
"""

from __future__ import annotations

from scenario.core import Rule, ScenarioBase
from scenario.scene import Scene, Zone


class EmergencyReconfigurationScenario(ScenarioBase):
    key = "emergency_reconfiguration"
    title = "Scenario 2 - Dynamic Emergency Zone and Static No-Fly Zone"
    mission_time = 170.0
    n_free = 4
    penalty_weight = 24.0

    def build_rules(self) -> list[Rule]:
        return [
            Rule("REQ-PLT-03", 1, "Emergency area exclusion", "Emerg", "#c0392b", "emergency"),
            Rule("REQ-PROX-06", 2, "Operational volume", "Geo", "#9b59b6", "geofence"),
            Rule("REQ-ZONE-03", 3, "Altitude ceiling", "Alt", "#e74c3c", "altitude"),
            Rule("REQ-ZONE-04", 3, "No-fly zone", "NFZ", "#e67e22", "no_fly"),
            Rule("REQ-EQP-07", 5, "Maximum speed", "Speed", "#8e44ad", "speed"),
        ]

    def build_scene(self) -> Scene:
        return Scene(
            env=(1100.0, 800.0, 170.0),
            start=(80.0, 120.0, 35.0),
            delivery=(1000.0, 650.0, 35.0),
            naive_wps=[
                (80.0, 120.0, 35.0),
                (510.0, 385.0, 135.0),
                (700.0, 500.0, 95.0),
                (1000.0, 650.0, 35.0),
            ],
            fallback_wps=[
                (80.0, 120.0, 35.0),
                (230.0, 85.0, 65.0),
                (330.0, 700.0, 65.0),
                (930.0, 735.0, 60.0),
                (1000.0, 650.0, 35.0),
            ],
            no_fly_zones=[
                Zone("Hospital helipad no-fly zone", 510.0, 385.0, 90.0, 150.0, "#e67e22", 0.35),
            ],
            emergency_zones=[
                Zone("Emergency response area", 700.0, 500.0, 135.0, 95.0, "#c0392b", 0.32, active_from=65.0),
            ],
        )
