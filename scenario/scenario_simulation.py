from __future__ import annotations

import argparse
import sys
from pathlib import Path


if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scenario.scenarios import get_scenarios


def main():
    scenarios = get_scenarios()

    parser = argparse.ArgumentParser(description="Generate drone rulebook scenario visualizations.")
    parser.add_argument("--scenario", choices=["all", *scenarios.keys()], default="all")
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parent / "output"))
    parser.add_argument("--show", action="store_true", help="Open the Plotly figure in a browser.")
    parser.add_argument("--no-opt", action="store_true", help="Use handcrafted compliant paths instead of scipy optimization.")
    parser.add_argument("--only-2d", action="store_true", help="Write only the static 2D SVG maps, skipping HTML generation.")
    args = parser.parse_args()

    selected = scenarios.values() if args.scenario == "all" else [scenarios[args.scenario]]
    for scenario in selected:
        scenario.run(Path(args.output_dir), show=args.show, no_opt=args.no_opt, write_html=not args.only_2d)


if __name__ == "__main__":
    main()
