from __future__ import annotations

from .battery_abort import BatteryAbortScenario
from .crowd_noise_night import CrowdNoiseNightScenario
from .emergency_reconfiguration import EmergencyReconfigurationScenario


SCENARIO_CLASSES = [
    CrowdNoiseNightScenario,
    EmergencyReconfigurationScenario,
    BatteryAbortScenario,
]


def get_scenarios():
    return {cls.key: cls() for cls in SCENARIO_CLASSES}

