# A Formal Rulebook for Autonomous Drone Operations
### ATIC 2026 — ETH Zürich

**Group members:** Mamoun Rhmari Tlemcani · Hadrien Legros · Ben Youssef Belkis · Saad Lahlou

**Supervisor:** Oliver Schön — oschoen@control.ee.ethz.ch  
**Submitted to:** Prof. Lars Lindemann — llindemann@control.ee.ethz.ch

---

## Project overview

This project translates EASA drone regulations (EU) 2019/947 and U-space regulation (EU) 2021/664 into a formal, machine-readable rulebook expressed in Signal Temporal Logic (STL), structured according to the priority-based rulebook framework of Censi et al. (2019).

The project contains three practical outputs:

- **Rulebook exports** (`STL_Rules.pdf`, `Rules_Plain_Language.pdf`) — 37+ rules extracted from EASA regulation, formalized in STL with 9 priority tiers. These PDF exports are ignored by git so they can be regenerated or shared separately.
- **Simulation** (`drone_simulation.py`) — A 3D animated compliance monitor and STL-guided trajectory optimiser running on real Zürich OSM building data.
- **Scenario pack** (`scenario/`) — reusable synthetic maps for illustrating selected rules, rule priorities, naive violations, and compliant trajectories.

---

## Simulation

The simulation demonstrates the rulebook on a parcel delivery mission over central Zürich. It shows two trajectories for the same mission:

- **Naive** (animated, colour-coded) — a rule-unaware direct route that violates altitude limits, no-fly zones, and crowd standoff rules.
- **Optimised** (static, green) — a trajectory found by an STL-guided optimiser that fixes all spatial violations while minimising path length.

### Rules implemented (7 of 37+)

| Rule ID | Tier | Description |
|---|---|---|
| REQ-PROX-01 | 1 | Crowd standoff — minimum distance from public assemblies |
| REQ-PROX-06 | 2 | Geofence containment — stay within operational volume |
| REQ-ZONE-03 | 3 | Altitude ceiling — max 120 m AGL |
| REQ-ZONE-04 | 3 | No-fly zone access — avoid restricted airspace |
| REQ-EQP-07 | 5 | Max speed — ≤ 19 m/s (C1-class drone) |
| REQ-EQP-03 | 5 | Battery warning — begin landing below 20% |
| REQ-EQP-12 | 7 | Noise limit — ≤ 70 dB community noise threshold |

### Zürich no-fly zones used
- UniversitätsSpital Helipad (47.3757°N, 8.5420°E) — radius 80 m
- ETH Zürich Campus (47.3769°N, 8.5477°E) — radius 60 m
- Zürich Hauptbahnhof (47.3779°N, 8.5402°E) — radius 70 m

---

## Scenario pack

The `scenario/` package provides small synthetic scenes that are easier to present than the full Zürich map. Each scenario is a class that combines:

- a **scene**: map size, start/delivery points, restricted areas, crowd zones, quiet zones, emergency zones, or safe landing areas;
- a **rule set**: 4-5 rules selected from the rulebook;
- a naive path and a compliant path generated with the same robustness-monitoring framework.

Current scenarios:

| Scenario | Main idea | Rules illustrated |
|---|---|---|
| `crowd_noise_night` | Crowd zones, residential quiet zone, day-to-night transition | crowd standoff, altitude, speed, night light, noise |
| `emergency_reconfiguration` | Static no-fly zone plus emergency area active mid-flight | emergency exclusion, geofence, altitude, no-fly zone, speed |
| `battery_abort` | Correct behavior is landing safely instead of completing delivery | geofence, no-fly zone, altitude, battery landing, speed |

Scenario code is split into reusable pieces:

```text
scenario/core.py                  # ScenarioBase, rules, monitoring, optimizer, report, animation
scenario/scene.py                 # Scene and Zone data models + Plotly scene geometry
scenario/scenarios/*.py           # One scenario class per file
scenario/scenario_simulation.py   # CLI runner
```

---

## Setup

```bash
pip install plotly numpy scipy
python drone_simulation.py
```

Run all synthetic scenarios:

```bash
python scenario/scenario_simulation.py
```

Run one scenario:

```bash
python scenario/scenario_simulation.py --scenario crowd_noise_night
```

Python 3.10+ required. The simulation fetches live building data from OpenStreetMap on startup (~5 seconds). The optimiser runs automatically after the buildings load (~5–10 seconds).

The main simulation opens an interactive Plotly page in your browser and prints a compliance table. The scenario runner writes interactive HTML files to `scenario/output/` and prints the same style of compliance comparison.

---

## Repository structure

```
drone_simulation.py      # Main simulation — monitoring + STL optimiser
scenario/                # Synthetic scenario framework and scenario classes
STL_Rules.pdf            # Ignored local export — formal STL rulebook
Rules_Plain_Language.pdf # Ignored local export — natural language rules
README.md
```

---

## Key results

| Rule | Naive trajectory | STL-optimised |
|---|---|---|
| Crowd standoff (Tier 1) | FAIL — min ρ = −81.7 m | PASS |
| Geofence (Tier 2) | PASS | PASS |
| Altitude ceiling (Tier 3) | FAIL — min ρ = −10.0 m | PASS — margin +59 m |
| No-fly zones (Tier 3) | FAIL — min ρ = −80.0 m | PASS — margin +48 m |
| Speed (Tier 5) | PASS | PASS |
| Battery warning (Tier 5) | FAIL — end-of-mission | FAIL — end-of-mission |
| Noise (Tier 7) | FAIL | PASS |
| **Overall compliance** | **72.0 %** | **92.8 %** |

Battery warning fails identically in both trajectories — this is physically correct. Routing cannot substitute for battery capacity; the drone correctly reaches the warning threshold late in a 4-minute urban delivery mission.

---

## References

1. Censi et al. "Liability, ethics, and culture-aware behavior specification using rulebooks." IEEE ICRA, 2019.
2. Rizaldi et al. "Formalising and monitoring traffic rules for autonomous vehicles in Isabelle/HOL." iFM, 2017.
3. EASA. Easy Access Rules for Unmanned Aircraft Systems (EU) 2019/947 and 2019/945, 2022.
4. Swiss FOCA. Drohnen — Regeln für den Betrieb, 2023.
