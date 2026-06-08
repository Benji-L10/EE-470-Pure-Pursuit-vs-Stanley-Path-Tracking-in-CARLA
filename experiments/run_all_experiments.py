#!/usr/bin/env python3
"""
Run the complete EE 470 CARLA experiment matrix.

Default matrix:
    controllers: pure_pursuit, stanley
    paths:       smooth_curvy, sharp_turns
    speeds:      5, 10, 15 m/s

This launches experiments/run_experiment.py once for every combination and
writes a batch report to data/batch_report.json.

Examples:

    python experiments/run_all_experiments.py

    python experiments/run_all_experiments.py \
        --map Town04 \
        --spectator-follow

    python experiments/run_all_experiments.py --dry-run
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER = REPO_ROOT / "experiments" / "run_experiment.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run all Pure Pursuit and Stanley CARLA experiments."
    )
    parser.add_argument(
        "--controllers",
        nargs="+",
        default=["pure_pursuit", "stanley"],
        choices=["pure_pursuit", "stanley"],
    )
    parser.add_argument(
        "--paths",
        nargs="+",
        default=["smooth_curvy", "sharp_turns"],
    )
    parser.add_argument(
        "--speeds",
        nargs="+",
        default=[5.0, 10.0, 15.0],
        type=float,
    )

    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--map", dest="map_name", default=None)
    parser.add_argument("--fixed-delta", default=0.05, type=float)
    parser.add_argument("--max-sim-time", default=120.0, type=float)
    parser.add_argument("--goal-tolerance", default=2.0, type=float)
    parser.add_argument("--max-cte", default=8.0, type=float)

    parser.add_argument("--spectator-follow", action="store_true")
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--pause-between-runs",
        default=2.0,
        type=float,
        help="Wall-clock pause between child processes [s].",
    )
    return parser.parse_args()


def build_command(
    args: argparse.Namespace,
    controller: str,
    path_name: str,
    speed: float,
) -> list[str]:
    path_csv = REPO_ROOT / "paths" / f"{path_name}.csv"

    command = [
        sys.executable,
        str(RUNNER),
        "--controller",
        controller,
        "--path-csv",
        str(path_csv),
        "--path-name",
        path_name,
        "--speed",
        f"{speed:g}",
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--fixed-delta",
        str(args.fixed_delta),
        "--max-sim-time",
        str(args.max_sim_time),
        "--goal-tolerance",
        str(args.goal_tolerance),
        "--max-cte",
        str(args.max_cte),
    ]

    if args.map_name:
        command.extend(["--map", args.map_name])

    if args.spectator_follow:
        command.append("--spectator-follow")

    if args.no_rendering:
        command.append("--no-rendering")

    return command


def main() -> int:
    args = parse_args()

    if not RUNNER.exists():
        raise FileNotFoundError(f"Experiment runner not found: {RUNNER}")

    jobs = [
        (controller, path_name, speed)
        for path_name in args.paths
        for speed in args.speeds
        for controller in args.controllers
    ]

    print(f"Prepared {len(jobs)} experiment runs.")
    for index, (controller, path_name, speed) in enumerate(jobs, start=1):
        print(f"  {index:02d}: {controller}, {path_name}, {speed:g} m/s")

    results: list[dict[str, Any]] = []
    batch_start = time.time()

    for index, (controller, path_name, speed) in enumerate(jobs, start=1):
        path_csv = REPO_ROOT / "paths" / f"{path_name}.csv"
        if not path_csv.exists():
            message = f"Missing path CSV: {path_csv}"
            if args.continue_on_error:
                print(f"\nSkipping run {index}: {message}", file=sys.stderr)
                results.append(
                    {
                        "controller": controller,
                        "path": path_name,
                        "speed_mps": speed,
                        "status": "skipped",
                        "reason": message,
                    }
                )
                continue
            raise FileNotFoundError(message)

        command = build_command(args, controller, path_name, speed)

        print("\n" + "=" * 78)
        print(
            f"Run {index}/{len(jobs)}: {controller} | "
            f"{path_name} | {speed:g} m/s"
        )
        print("Command:")
        print(" ".join(command))
        print("=" * 78)

        if args.dry_run:
            results.append(
                {
                    "controller": controller,
                    "path": path_name,
                    "speed_mps": speed,
                    "status": "dry_run",
                }
            )
            continue

        run_start = time.time()
        completed = subprocess.run(command, cwd=REPO_ROOT)
        elapsed = time.time() - run_start

        status = "success" if completed.returncode == 0 else "failed"
        result = {
            "controller": controller,
            "path": path_name,
            "speed_mps": speed,
            "status": status,
            "return_code": completed.returncode,
            "wall_time_s": elapsed,
        }
        results.append(result)

        if completed.returncode != 0 and not args.continue_on_error:
            print(
                "\nStopping because a run failed. Use --continue-on-error "
                "to attempt the remaining cases.",
                file=sys.stderr,
            )
            break

        if index < len(jobs) and args.pause_between_runs > 0:
            time.sleep(args.pause_between_runs)

    report = {
        "total_requested": len(jobs),
        "total_attempted": len(results),
        "successes": sum(result["status"] == "success" for result in results),
        "failures": sum(result["status"] == "failed" for result in results),
        "elapsed_wall_time_s": time.time() - batch_start,
        "runs": results,
    }

    report_path = REPO_ROOT / "data" / "batch_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w") as file:
        json.dump(report, file, indent=2)

    print("\nBatch complete.")
    print(json.dumps(report, indent=2))
    print(f"Batch report: {report_path}")

    return 1 if report["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
