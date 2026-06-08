#!/usr/bin/env python3
"""
Generate two CARLA reference paths for the EE 470 controller comparison.

The script searches CARLA spawn-point pairs, traces road routes with CARLA's
GlobalRoutePlanner, computes geometric features, and selects:

1. smooth_curvy.csv:
   A long route with distributed curvature and no extremely abrupt heading jump.

2. sharp_turns.csv:
   A long route containing one or more relatively sharp direction changes.

Run from the repository root while the CARLA server is open:

    python experiments/generate_paths.py --map Town04

You may also explicitly choose spawn-point pairs:

    python experiments/generate_paths.py \
        --smooth-start 12 --smooth-goal 74 \
        --sharp-start 23 --sharp-goal 105

The generated CSV files use CARLA world coordinates and contain:
    x,y,yaw,road_id,lane_id
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError as exc:
    raise SystemExit(
        "Could not import CARLA or GlobalRoutePlanner. Make sure CARLA's "
        "PythonAPI and the 'agents' package are on PYTHONPATH."
    ) from exc


REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class RouteCandidate:
    start_index: int
    goal_index: int
    route: list[tuple[Any, Any]]
    length_m: float
    total_abs_turn_deg: float
    max_turn_deg: float
    p90_turn_deg: float
    mean_turn_deg: float

    @property
    def points(self) -> np.ndarray:
        return np.asarray(
            [
                [waypoint.transform.location.x, waypoint.transform.location.y]
                for waypoint, _road_option in self.route
            ],
            dtype=float,
        )


def wrap_to_pi(angle: np.ndarray | float) -> np.ndarray | float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def route_metrics(route: list[tuple[Any, Any]]) -> tuple[float, float, float, float, float]:
    points = np.asarray(
        [
            [waypoint.transform.location.x, waypoint.transform.location.y]
            for waypoint, _road_option in route
        ],
        dtype=float,
    )

    if len(points) < 3:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    segments = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(segments, axis=1)

    valid = segment_lengths > 1e-6
    segments = segments[valid]
    segment_lengths = segment_lengths[valid]

    if len(segments) < 2:
        return float(np.sum(segment_lengths)), 0.0, 0.0, 0.0, 0.0

    headings = np.arctan2(segments[:, 1], segments[:, 0])
    heading_changes = np.abs(wrap_to_pi(np.diff(headings)))
    heading_changes_deg = np.degrees(heading_changes)

    length_m = float(np.sum(segment_lengths))
    total_abs_turn_deg = float(np.sum(heading_changes_deg))
    max_turn_deg = float(np.max(heading_changes_deg))
    p90_turn_deg = float(np.percentile(heading_changes_deg, 90))
    mean_turn_deg = float(np.mean(heading_changes_deg))

    return (
        length_m,
        total_abs_turn_deg,
        max_turn_deg,
        p90_turn_deg,
        mean_turn_deg,
    )


def make_candidate(
    planner: GlobalRoutePlanner,
    spawn_points: list[carla.Transform],
    start_index: int,
    goal_index: int,
) -> RouteCandidate | None:
    if start_index == goal_index:
        return None

    start = spawn_points[start_index].location
    goal = spawn_points[goal_index].location

    route = planner.trace_route(start, goal)
    if len(route) < 3:
        return None

    metrics = route_metrics(route)
    return RouteCandidate(
        start_index=start_index,
        goal_index=goal_index,
        route=route,
        length_m=metrics[0],
        total_abs_turn_deg=metrics[1],
        max_turn_deg=metrics[2],
        p90_turn_deg=metrics[3],
        mean_turn_deg=metrics[4],
    )


def smooth_score(candidate: RouteCandidate) -> float:
    """
    Prefer a long route with meaningful distributed curvature, but penalize
    isolated abrupt turns.
    """
    curvature_density = candidate.total_abs_turn_deg / max(candidate.length_m, 1.0)
    return (
        0.025 * candidate.length_m
        + 2.0 * curvature_density
        - 0.08 * candidate.max_turn_deg
        - 0.04 * candidate.p90_turn_deg
    )


def sharp_score(candidate: RouteCandidate) -> float:
    """Prefer long routes with large direction changes."""
    curvature_density = candidate.total_abs_turn_deg / max(candidate.length_m, 1.0)
    return (
        0.015 * candidate.length_m
        + 0.16 * candidate.max_turn_deg
        + 0.08 * candidate.p90_turn_deg
        + 1.5 * curvature_density
    )


def choose_routes(
    planner: GlobalRoutePlanner,
    spawn_points: list[carla.Transform],
    samples: int,
    min_length: float,
    max_length: float,
    seed: int,
) -> tuple[RouteCandidate, RouteCandidate]:
    rng = random.Random(seed)
    pair_set: set[tuple[int, int]] = set()
    number_of_spawns = len(spawn_points)

    maximum_pairs = number_of_spawns * max(number_of_spawns - 1, 0)
    target_samples = min(samples, maximum_pairs)

    while len(pair_set) < target_samples:
        pair = (
            rng.randrange(number_of_spawns),
            rng.randrange(number_of_spawns),
        )
        if pair[0] != pair[1]:
            pair_set.add(pair)

    candidates: list[RouteCandidate] = []

    for number, (start_index, goal_index) in enumerate(sorted(pair_set), start=1):
        try:
            candidate = make_candidate(
                planner,
                spawn_points,
                start_index,
                goal_index,
            )
        except Exception as exc:
            print(
                f"Skipping route {start_index}->{goal_index}: {exc}",
                file=sys.stderr,
            )
            continue

        if candidate is None:
            continue

        if min_length <= candidate.length_m <= max_length:
            candidates.append(candidate)

        if number % 25 == 0:
            print(f"Evaluated {number}/{len(pair_set)} spawn-point pairs...")

    if len(candidates) < 2:
        raise RuntimeError(
            "Not enough valid candidate routes. Increase --samples, reduce "
            "--min-length, or increase --max-length."
        )

    # Smooth route: reject candidates with extremely abrupt sampled changes
    # when alternatives exist.
    smooth_pool = [
        candidate for candidate in candidates if candidate.max_turn_deg <= 35.0
    ]
    if not smooth_pool:
        smooth_pool = candidates

    smooth = max(smooth_pool, key=smooth_score)

    sharp_pool = [
        candidate
        for candidate in candidates
        if (candidate.start_index, candidate.goal_index)
        != (smooth.start_index, smooth.goal_index)
    ]
    sharp = max(sharp_pool, key=sharp_score)

    return smooth, sharp


def write_route_csv(candidate: RouteCandidate, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", newline="") as file:
        fieldnames = ["x", "y", "yaw", "road_id", "lane_id"]
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        for waypoint, _road_option in candidate.route:
            transform = waypoint.transform
            writer.writerow(
                {
                    "x": f"{transform.location.x:.6f}",
                    "y": f"{transform.location.y:.6f}",
                    "yaw": f"{math.radians(transform.rotation.yaw):.8f}",
                    "road_id": waypoint.road_id,
                    "lane_id": waypoint.lane_id,
                }
            )


def print_candidate(label: str, candidate: RouteCandidate, output_path: Path) -> None:
    print(f"\n{label}")
    print(f"  spawn pair: {candidate.start_index} -> {candidate.goal_index}")
    print(f"  route length: {candidate.length_m:.1f} m")
    print(f"  total absolute heading change: {candidate.total_abs_turn_deg:.1f} deg")
    print(f"  maximum sampled heading change: {candidate.max_turn_deg:.1f} deg")
    print(f"  90th percentile heading change: {candidate.p90_turn_deg:.1f} deg")
    print(f"  output: {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate smooth-curvy and sharp-turn CARLA paths."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout", default=20.0, type=float)
    parser.add_argument(
        "--map",
        dest="map_name",
        default=None,
        help="Optional CARLA map to load, such as Town04.",
    )
    parser.add_argument("--resolution", default=1.0, type=float)
    parser.add_argument("--samples", default=250, type=int)
    parser.add_argument("--min-length", default=120.0, type=float)
    parser.add_argument("--max-length", default=500.0, type=float)
    parser.add_argument("--seed", default=470, type=int)

    parser.add_argument("--smooth-start", type=int, default=None)
    parser.add_argument("--smooth-goal", type=int, default=None)
    parser.add_argument("--sharp-start", type=int, default=None)
    parser.add_argument("--sharp-goal", type=int, default=None)

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "paths",
    )
    return parser.parse_args()


def explicit_pair_supplied(start: int | None, goal: int | None) -> bool:
    if (start is None) != (goal is None):
        raise ValueError("Both start and goal indices must be supplied together.")
    return start is not None and goal is not None


def main() -> int:
    args = parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    if args.map_name:
        world = client.load_world(args.map_name)
    else:
        world = client.get_world()

    world_map = world.get_map()
    spawn_points = world_map.get_spawn_points()

    if len(spawn_points) < 2:
        raise RuntimeError("The loaded map does not provide enough spawn points.")

    print(f"Loaded map: {world_map.name}")
    print(f"Spawn points: {len(spawn_points)}")
    print(f"Route sampling resolution: {args.resolution:.2f} m")

    planner = GlobalRoutePlanner(world_map, args.resolution)

    use_explicit_smooth = explicit_pair_supplied(
        args.smooth_start,
        args.smooth_goal,
    )
    use_explicit_sharp = explicit_pair_supplied(
        args.sharp_start,
        args.sharp_goal,
    )

    if use_explicit_smooth or use_explicit_sharp:
        if not (use_explicit_smooth and use_explicit_sharp):
            raise ValueError(
                "For explicit mode, provide both the smooth and sharp spawn pairs."
            )

        smooth = make_candidate(
            planner,
            spawn_points,
            args.smooth_start,
            args.smooth_goal,
        )
        sharp = make_candidate(
            planner,
            spawn_points,
            args.sharp_start,
            args.sharp_goal,
        )

        if smooth is None or sharp is None:
            raise RuntimeError("One of the explicit routes could not be traced.")
    else:
        smooth, sharp = choose_routes(
            planner=planner,
            spawn_points=spawn_points,
            samples=args.samples,
            min_length=args.min_length,
            max_length=args.max_length,
            seed=args.seed,
        )

    smooth_path = args.output_dir / "smooth_curvy.csv"
    sharp_path = args.output_dir / "sharp_turns.csv"

    write_route_csv(smooth, smooth_path)
    write_route_csv(sharp, sharp_path)

    print_candidate("Selected smooth-curvy route:", smooth, smooth_path)
    print_candidate("Selected sharp-turn route:", sharp, sharp_path)

    print(
        "\nInspect both paths in CARLA before collecting final data. "
        "The automatic scores select geometrically different routes, but visual "
        "inspection is still important."
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
