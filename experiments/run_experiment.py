#!/usr/bin/env python3
"""
Run one repeatable CARLA path-tracking experiment.

Expected project structure:
    controllers/
        pure_pursuit.py   -> PurePursuitController
        stanley.py        -> StanleyController
    paths/
        smooth_curvy.csv
        sharp_turns.csv
    experiments/
        run_experiment.py

Path CSV format:
    x,y
    12.3,45.6
    13.1,45.9
    ...

The path coordinates must be CARLA world coordinates.

Example:
    python experiments/run_experiment.py \
        --controller pure_pursuit \
        --path-csv paths/smooth_curvy.csv \
        --path-name smooth_curvy \
        --speed 5 \
        --map Town04
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    import carla
except ImportError as exc:
    raise SystemExit(
        "Could not import the CARLA Python API. Add CARLA's PythonAPI package "
        "to PYTHONPATH or install the wheel matching your CARLA version."
    ) from exc

# Allow execution from either the repository root or experiments/ directory.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from controllers.pure_pursuit import PurePursuitController
from controllers.stanley import StanleyController


@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float
    speed: float

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "speed": self.speed}


class PIController:
    """Simple longitudinal PI controller with anti-windup."""

    def __init__(self, kp: float = 0.45, ki: float = 0.08, integral_limit: float = 5.0) -> None:
        self.kp = kp
        self.ki = ki
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0

    def compute(self, target_speed: float, speed: float, dt: float) -> tuple[float, float]:
        error = target_speed - speed
        self.integral += error * dt
        self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))

        command = self.kp * error + self.ki * self.integral

        if command >= 0.0:
            return float(np.clip(command, 0.0, 1.0)), 0.0
        return 0.0, float(np.clip(-command, 0.0, 1.0))


class CollisionMonitor:
    def __init__(self) -> None:
        self.collided = False
        self.impulse = 0.0

    def callback(self, event: Any) -> None:
        impulse = event.normal_impulse
        self.impulse = math.sqrt(
            impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z
        )
        self.collided = True


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def load_path(path_file: Path) -> np.ndarray:
    if not path_file.exists():
        raise FileNotFoundError(f"Path CSV not found: {path_file}")

    points: list[tuple[float, float]] = []
    with path_file.open("r", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or not {"x", "y"}.issubset(reader.fieldnames):
            raise ValueError("Path CSV must contain columns named 'x' and 'y'.")

        for row_number, row in enumerate(reader, start=2):
            try:
                points.append((float(row["x"]), float(row["y"])))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid x/y value in {path_file} on row {row_number}."
                ) from exc

    if len(points) < 3:
        raise ValueError("Path must contain at least three points.")

    return np.asarray(points, dtype=float)


def cumulative_distance(path: np.ndarray) -> np.ndarray:
    segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segment_lengths)))


def extract_vehicle_state(vehicle: carla.Vehicle) -> VehicleState:
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    speed = math.sqrt(
        velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z
    )
    return VehicleState(
        x=float(transform.location.x),
        y=float(transform.location.y),
        yaw=math.radians(float(transform.rotation.yaw)),
        speed=float(speed),
    )


def nearest_path_data(
    position: np.ndarray,
    path: np.ndarray,
    previous_index: int,
    search_window: int = 80,
) -> tuple[int, np.ndarray, float, float]:
    """Return nearest forward index, reference point, path yaw, and signed CTE."""
    start = max(0, previous_index - 5)
    end = min(len(path), previous_index + search_window)

    local_path = path[start:end]
    local_idx = int(np.argmin(np.linalg.norm(local_path - position, axis=1)))
    idx = start + local_idx

    if idx < len(path) - 1:
        tangent = path[idx + 1] - path[idx]
    else:
        tangent = path[idx] - path[idx - 1]

    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm < 1e-9:
        raise RuntimeError(f"Duplicate or degenerate path points near index {idx}.")

    tangent_unit = tangent / tangent_norm
    path_yaw = math.atan2(float(tangent_unit[1]), float(tangent_unit[0]))
    normal = np.array([-tangent_unit[1], tangent_unit[0]])
    signed_cte = float((position - path[idx]) @ normal)

    return idx, path[idx], path_yaw, signed_cte


def choose_spawn_transform(
    world_map: carla.Map,
    path: np.ndarray,
    spawn_index: int | None,
) -> carla.Transform:
    spawn_points = world_map.get_spawn_points()
    if not spawn_points:
        raise RuntimeError("This CARLA map has no recommended spawn points.")

    if spawn_index is not None:
        if spawn_index < 0 or spawn_index >= len(spawn_points):
            raise IndexError(
                f"spawn-index {spawn_index} is outside [0, {len(spawn_points) - 1}]."
            )
        return spawn_points[spawn_index]

    path_start = path[0]
    return min(
        spawn_points,
        key=lambda transform: (
            (transform.location.x - path_start[0]) ** 2
            + (transform.location.y - path_start[1]) ** 2
        ),
    )


def get_vehicle_max_steer_angle(vehicle: carla.Vehicle, fallback_degrees: float) -> float:
    """Return an approximate physical steering-angle limit in radians."""
    try:
        physics = vehicle.get_physics_control()
        angles = [
            abs(float(wheel.max_steer_angle))
            for wheel in physics.wheels
            if abs(float(wheel.max_steer_angle)) > 1e-3
        ]
        if angles:
            return math.radians(max(angles))
    except RuntimeError:
        pass

    return math.radians(fallback_degrees)


def build_controller(args: argparse.Namespace):
    max_steer_rad = math.radians(args.max_steer_angle_deg)

    if args.controller == "pure_pursuit":
        return PurePursuitController(
            wheelbase=args.wheelbase,
            lookahead_gain=args.lookahead_gain,
            min_lookahead=args.min_lookahead,
            max_steer=max_steer_rad,
        )

    return StanleyController(
        wheelbase=args.wheelbase,
        k=args.stanley_gain,
        ks=args.stanley_softening,
        max_steer=max_steer_rad,
    )


def default_output_path(args: argparse.Namespace) -> Path:
    speed_text = f"{args.speed:g}".replace(".", "p")
    return REPO_ROOT / "data" / args.path_name / f"{args.controller}_{speed_text}ms.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one Pure Pursuit or Stanley experiment in CARLA."
    )

    parser.add_argument("--controller", required=True, choices=("pure_pursuit", "stanley"))
    parser.add_argument("--path-csv", required=True, type=Path)
    parser.add_argument("--path-name", default="custom_path")
    parser.add_argument("--speed", required=True, type=float, help="Target speed [m/s].")

    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout", default=15.0, type=float)
    parser.add_argument(
        "--map", dest="map_name", default=None,
        help="Optional CARLA map, for example Town04. Omit to use the loaded map."
    )

    parser.add_argument("--vehicle-filter", default="vehicle.tesla.model3")
    parser.add_argument("--spawn-index", default=None, type=int)
    parser.add_argument("--fixed-delta", default=0.05, type=float)
    parser.add_argument("--max-sim-time", default=120.0, type=float)
    parser.add_argument("--goal-tolerance", default=2.0, type=float)
    parser.add_argument("--max-cte", default=8.0, type=float)
    parser.add_argument("--warmup-ticks", default=10, type=int)

    parser.add_argument("--wheelbase", default=2.875, type=float)
    parser.add_argument("--max-steer-angle-deg", default=35.0, type=float)
    parser.add_argument("--lookahead-gain", default=0.5, type=float)
    parser.add_argument("--min-lookahead", default=3.0, type=float)
    parser.add_argument("--stanley-gain", default=0.8, type=float)
    parser.add_argument("--stanley-softening", default=1.0, type=float)
    parser.add_argument("--speed-kp", default=0.45, type=float)
    parser.add_argument("--speed-ki", default=0.08, type=float)

    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--spectator-follow", action="store_true")

    args = parser.parse_args()
    if args.speed <= 0.0:
        parser.error("--speed must be positive.")
    if args.fixed_delta <= 0.0:
        parser.error("--fixed-delta must be positive.")
    if args.max_sim_time <= 0.0:
        parser.error("--max-sim-time must be positive.")
    return args


def run_experiment(args: argparse.Namespace) -> dict[str, Any]:
    path = load_path(args.path_csv.resolve())

    print("Path points:", len(path))
    print("First point:", path[0])
    print("Last point:", path[-1])
    
    path_progress = cumulative_distance(path)
    controller = build_controller(args)
    speed_controller = PIController(kp=args.speed_kp, ki=args.speed_ki)

    output_path = (args.output or default_output_path(args)).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world: carla.World | None = None
    original_settings: carla.WorldSettings | None = None
    vehicle: carla.Vehicle | None = None
    collision_sensor: carla.Sensor | None = None
    collision_monitor = CollisionMonitor()
    rows: list[dict[str, float | int | str | bool]] = []
    termination_reason = "unknown"

    try:
        world = client.load_world(args.map_name) if args.map_name else client.get_world()
        original_settings = world.get_settings()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.fixed_delta
        settings.no_rendering_mode = args.no_rendering
        world.apply_settings(settings)
        world.tick()

        world_map = world.get_map()
        spawn_transform = choose_spawn_transform(world_map, path, args.spawn_index)

        print(
            "Spawn:",
            spawn_transform.location.x,
            spawn_transform.location.y
        )
        print(
            "Path start:",
            path[0][0],
            path[0][1]
        )

        blueprint_library = world.get_blueprint_library()
        matching_blueprints = blueprint_library.filter(args.vehicle_filter)
        if not matching_blueprints:
            raise RuntimeError(f"No vehicle blueprint matched '{args.vehicle_filter}'.")

        vehicle_bp = matching_blueprints[0]
        if vehicle_bp.has_attribute("role_name"):
            vehicle_bp.set_attribute("role_name", "hero")

        vehicle = world.try_spawn_actor(vehicle_bp, spawn_transform)
        if vehicle is None:
            raise RuntimeError("Vehicle spawn failed. Try another --spawn-index or clear the map.")

        collision_bp = blueprint_library.find("sensor.other.collision")
        collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
        collision_sensor.listen(collision_monitor.callback)

        physical_max_steer = get_vehicle_max_steer_angle(
            vehicle, fallback_degrees=args.max_steer_angle_deg
        )

        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
        for _ in range(args.warmup_ticks):
            world.tick()

        previous_idx = 0
        number_of_steps = int(math.ceil(args.max_sim_time / args.fixed_delta))

        print(f"Running {args.controller} on {args.path_name} at {args.speed:.2f} m/s")
        print(f"Output: {output_path}")
        print(
            f"CARLA map: {world_map.name}; fixed dt: {args.fixed_delta:.3f} s; "
            f"vehicle steering scale: {math.degrees(physical_max_steer):.1f} deg"
        )

        for step in range(number_of_steps):
            state = extract_vehicle_state(vehicle)
            position = np.array([state.x, state.y])

            closest_idx, reference_point, path_yaw, signed_cte = nearest_path_data(
                position, path, previous_idx
            )
            previous_idx = max(previous_idx, closest_idx)
            heading_error = wrap_to_pi(path_yaw - state.yaw)

            steering_angle = float(controller.compute_control(state.as_dict(), path))
            normalized_steer = float(
                np.clip(steering_angle / physical_max_steer, -1.0, 1.0)
            )
            throttle, brake = speed_controller.compute(
                target_speed=args.speed, speed=state.speed, dt=args.fixed_delta
            )

            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=throttle,
                    steer=normalized_steer,
                    brake=brake,
                    hand_brake=False,
                    reverse=False,
                    manual_gear_shift=False,
                )
            )

            sim_time = step * args.fixed_delta
            distance_to_goal = float(np.linalg.norm(position - path[-1]))

            rows.append({
                "time": sim_time,
                "frame": step,
                "controller": args.controller,
                "path_name": args.path_name,
                "target_speed": args.speed,
                "x": state.x,
                "y": state.y,
                "yaw": state.yaw,
                "speed": state.speed,
                "steering_angle_rad": steering_angle,
                "steer_normalized": normalized_steer,
                "throttle": throttle,
                "brake": brake,
                "reference_x": float(reference_point[0]),
                "reference_y": float(reference_point[1]),
                "closest_path_index": closest_idx,
                "path_progress_m": float(path_progress[closest_idx]),
                "cross_track_error": signed_cte,
                "heading_error": heading_error,
                "distance_to_goal": distance_to_goal,
                "collision": collision_monitor.collided,
            })

            if args.spectator_follow:
                transform = vehicle.get_transform()
                forward = transform.get_forward_vector()
                spectator_location = transform.location - 8.0 * forward
                spectator_location.z += 4.0
                spectator_rotation = carla.Rotation(
                    pitch=-15.0, yaw=transform.rotation.yaw, roll=0.0
                )
                world.get_spectator().set_transform(
                    carla.Transform(spectator_location, spectator_rotation)
                )

            world.tick()

            if collision_monitor.collided:
                termination_reason = "collision"
                break
            if abs(signed_cte) > args.max_cte:
                termination_reason = "cross_track_error_limit"
                break
            if closest_idx >= len(path) - 3 and distance_to_goal <= args.goal_tolerance:
                termination_reason = "goal_reached"
                break
        else:
            termination_reason = "time_limit"

        if not rows:
            raise RuntimeError("Experiment ended before any samples were logged.")

        with output_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        cte = np.asarray([float(row["cross_track_error"]) for row in rows])
        heading = np.asarray([float(row["heading_error"]) for row in rows])
        steer = np.asarray([float(row["steering_angle_rad"]) for row in rows])
        speed = np.asarray([float(row["speed"]) for row in rows])
        steering_rate = np.diff(steer) / args.fixed_delta if len(steer) > 1 else np.asarray([0.0])

        summary: dict[str, Any] = {
            "controller": args.controller,
            "path_name": args.path_name,
            "target_speed_mps": args.speed,
            "samples": len(rows),
            "simulated_time_s": float(rows[-1]["time"]),
            "termination_reason": termination_reason,
            "collision_impulse": collision_monitor.impulse,
            "mean_abs_cte_m": float(np.mean(np.abs(cte))),
            "rms_cte_m": float(np.sqrt(np.mean(cte**2))),
            "max_abs_cte_m": float(np.max(np.abs(cte))),
            "mean_abs_heading_error_rad": float(np.mean(np.abs(heading))),
            "max_abs_heading_error_rad": float(np.max(np.abs(heading))),
            "mean_speed_mps": float(np.mean(speed)),
            "mean_abs_speed_error_mps": float(np.mean(np.abs(args.speed - speed))),
            "mean_abs_steering_rate_rad_s": float(np.mean(np.abs(steering_rate))),
            "rms_steering_rate_rad_s": float(np.sqrt(np.mean(steering_rate**2))),
            "csv_path": str(output_path),
        }

        summary_path = output_path.with_suffix(".summary.json")
        with summary_path.open("w") as file:
            json.dump(summary, file, indent=2)

        print(json.dumps(summary, indent=2))
        print(f"Summary: {summary_path}")
        return summary

    finally:
        if vehicle is not None:
            try:
                vehicle.apply_control(
                    carla.VehicleControl(
                        throttle=0.0, brake=1.0, steer=0.0, hand_brake=True
                    )
                )
            except RuntimeError:
                pass

        if collision_sensor is not None:
            try:
                collision_sensor.stop()
            except RuntimeError:
                pass
            collision_sensor.destroy()

        if vehicle is not None:
            vehicle.destroy()

        if world is not None and original_settings is not None:
            world.apply_settings(original_settings)


def main() -> int:
    args = parse_args()
    try:
        run_experiment(args)
    except KeyboardInterrupt:
        print("\nExperiment interrupted by user.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nExperiment failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
