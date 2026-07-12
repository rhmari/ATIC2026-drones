# Scenario Pack

Synthetic scenarios for illustrating the rulebook without administrative rules or OpenStreetMap dependency.

Dependencies are the same as the main demo:

```bash
pip install plotly numpy scipy
```

Run all scenarios:

```bash
python scenario/scenario_simulation.py
```

Run one scenario and open it:

```bash
python scenario/scenario_simulation.py --scenario crowd_noise_night --show
```

Generate only the static 2D maps:

```bash
python scenario/scenario_simulation.py --only-2d
```

Structure:

- `scene.py`: reusable map primitives, zones, scene geometry, and Plotly scene drawing.
- `core.py`: reusable scenario base class, robustness evaluation, optimizer, report, and animation.
- `scenarios/*.py`: one class per scenario; each class combines a scene with its rules.
- `scenario_simulation.py`: thin command-line runner.

Available scenarios:

- `crowd_noise_night`: crowd standoff, quiet-zone noise, speed, altitude, night lighting.
- `emergency_reconfiguration`: dynamic emergency response area, static no-fly zone, geofence, altitude, speed.
- `battery_abort`: restricted airspace plus battery-triggered safe landing instead of completing the delivery.

The generated HTML files are written to `scenario/output/`.

Static 2D SVG maps are also written to `scenario/output_2d/`. These show only
the world, rule areas, naive trajectory, and compliant trajectory, without the
interactive Plotly view.
