from __future__ import annotations

import argparse
import sys
from pathlib import Path


if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scenario.scenarios import get_scenarios


def main():
    scenarios = get_scenarios()

    parser = argparse.ArgumentParser(description="Generate rule-constrained drone scenario visualizations.")
    parser.add_argument("--scenario", choices=["all", *scenarios.keys()], default="all")
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent / "output"))
    parser.add_argument("--show", action="store_true", help="Open the Plotly figure in a browser.")
    parser.add_argument("--no-opt", action="store_true", help="Use handcrafted compliant paths instead of scipy optimization.")
    parser.add_argument("--only-2d", action="store_true", help="Write only the static 2D SVG maps, skipping HTML generation.")
    parser.add_argument(
        "--crowd-buffer",
        type=float,
        default=None,
        metavar="METRES",
        help="Generate a separate crowd_noise_night result with this additional crowd-clearance margin.",
    )
    args = parser.parse_args()

    selected = scenarios.values() if args.scenario == "all" else [scenarios[args.scenario]]
    for scenario in selected:
        if args.crowd_buffer is not None and scenario.key == "crowd_noise_night":
            if args.crowd_buffer < 0:
                parser.error("--crowd-buffer must be non-negative")
            scenario.rule_margins = {"REQ-PROX-01": args.crowd_buffer}
            suffix = f"_buffer_{args.crowd_buffer:g}m"
            scenario.key += suffix
            scenario.title += f" (+{args.crowd_buffer:g} m Crowd Buffer)"
        scenario.run(Path(args.output_dir), show=args.show, no_opt=args.no_opt, write_html=not args.only_2d)


if __name__ == "__main__":
    main()
