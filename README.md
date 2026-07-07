# A Formal Rulebook for Autonomous Drone Operations
### ATIC 2026 — ETH Zürich

**Group members:** Mamoun Rhmari Tlemcani · Hadrien Legros · Ben Youssef Belkis · Saad Lahlou

**Supervisor:** Oliver Schön — oschoen@control.ee.ethz.ch  
**Submitted to:** Prof. Lars Lindemann — llindemann@control.ee.ethz.ch

---

## Project overview

This project translates EASA drone regulations (EU) 2019/947 and U-space regulation (EU) 2021/664 into a formal, machine-readable rulebook expressed in Signal Temporal Logic (STL), structured according to the priority-based rulebook framework of Censi et al. (2019).

The repository contains two deliverables:

- **Rulebook** (`STL_Rules.pdf`, `Rules_Plain_Language.pdf`) — 37+ rules extracted from EASA regulation, formalized in STL with 9 priority tiers.
- **Simulation** (`drone_simulation.py`) — A 3D animated compliance monitor and STL-guided trajectory optimiser running on real Zürich OSM building data.

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

## Setup

```bash
pip install plotly numpy scipy
python drone_simulation.py
```

Python 3.10+ required. The simulation fetches live building data from OpenStreetMap on startup (~5 seconds). The optimiser runs automatically after the buildings load (~5–10 seconds).

The output is an interactive HTML page that opens in your browser, plus a compliance comparison table printed to the console.

---

## Repository structure

```
drone_simulation.py      # Main simulation — monitoring + STL optimiser
STL_Rules.pdf            # Formal STL rulebook (9 tiers, 37+ rules)
Rules_Plain_Language.pdf # Natural language rules extracted from EASA regulation
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
