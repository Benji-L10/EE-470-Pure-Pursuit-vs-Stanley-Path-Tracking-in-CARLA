#!/usr/bin/env python3
"""
Create report-ready comparison figures for Pure Pursuit vs. Stanley.

Expected data layout:
    data/
    ├── smooth_curvy/
    │   ├── pure_pursuit_5ms.csv
    │   ├── pure_pursuit_10ms.csv
    │   ├── pure_pursuit_15ms.csv
    │   ├── stanley_5ms.csv
    │   ├── stanley_10ms.csv
    │   └── stanley_15ms.csv
    └── sharp_turns/
        ├── pure_pursuit_5ms.csv
        ├── pure_pursuit_10ms.csv
        ├── pure_pursuit_15ms.csv
        ├── stanley_5ms.csv
        ├── stanley_10ms.csv
        └── stanley_15ms.csv

Default output:
    plots/report/
    ├── trajectory_comparison_10ms.png
    ├── cross_track_error_10ms.png
    ├── metric_summary.png
    └── all_summary_metrics.csv

Example:
    python utils/plotting.py --data-dir data --speed 10 --output-dir plots/report
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PATHS = {
    "smooth_curvy": "Smooth curvy path",
    "sharp_turns": "Sharp-turn path",
}

CONTROLLERS = {
    "pure_pursuit": "Pure Pursuit",
    "stanley": "Stanley",
}

DEFAULT_SPEEDS = (5, 10, 15)

REQUIRED_COLUMNS = {
    "time",
    "x",
    "y",
    "speed",
    "target_speed",
    "steering_angle_rad",
    "reference_x",
    "reference_y",
    "cross_track_error",
    "heading_error",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate compact IEEE-report figures from CARLA CSV data."
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("data"),
        help="Root directory containing smooth_curvy/ and sharp_turns/.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("plots/report"),
        help="Directory for report-ready figures and summary metrics.",
    )
    parser.add_argument(
        "--speed",
        type=int,
        choices=DEFAULT_SPEEDS,
        default=10,
        help="Representative speed used in trajectory and CTE figures.",
    )
    parser.add_argument(
        "--speeds",
        type=int,
        nargs="+",
        default=list(DEFAULT_SPEEDS),
        help="Speeds included in the summary table and metric bar plot.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="Output resolution.",
    )
    return parser.parse_args()


def csv_path(data_dir: Path, path_name: str, controller: str, speed: int) -> Path:
    return data_dir / path_name / f"{controller}_{speed}ms.csv"


def load_run(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Missing experiment CSV: {path}")

    df = pd.read_csv(path)
    missing = REQUIRED_COLUMNS.difference(df.columns)

    if missing:
        raise ValueError(
            f"{path} is missing required columns: {sorted(missing)}"
        )

    return df


def clean_reference(df: pd.DataFrame) -> pd.DataFrame:
    """
    Return reference points in route order without plotting the same closest
    waypoint hundreds of times.
    """
    columns = ["reference_x", "reference_y"]

    if "closest_path_index" in df.columns:
        columns.append("closest_path_index")
        reference = df[columns].drop_duplicates(
            subset="closest_path_index",
            keep="first",
        )
        reference = reference.sort_values("closest_path_index")
    else:
        reference = df[columns].drop_duplicates(keep="first")

    return reference


def style_axis(ax: plt.Axes) -> None:
    ax.grid(True, alpha=0.30, linewidth=0.7)
    ax.tick_params(labelsize=8)


def save_figure(fig: plt.Figure, output_path: Path, dpi: int) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {output_path}")


def make_trajectory_figure(
    data_dir: Path,
    speed: int,
    output_dir: Path,
    dpi: int,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.25))

    for ax, (path_name, path_title) in zip(axes, PATHS.items()):
        pp = load_run(csv_path(data_dir, path_name, "pure_pursuit", speed))
        stanley = load_run(csv_path(data_dir, path_name, "stanley", speed))
        reference = clean_reference(pp)

        ax.plot(
            reference["reference_x"],
            reference["reference_y"],
            linestyle="--",
            linewidth=1.6,
            label="Reference",
        )
        ax.plot(
            pp["x"],
            pp["y"],
            linewidth=1.45,
            label="Pure Pursuit",
        )
        ax.plot(
            stanley["x"],
            stanley["y"],
            linewidth=1.45,
            label="Stanley",
        )

        ax.set_title(path_title, fontsize=9)
        ax.set_xlabel("CARLA x position [m]", fontsize=8)
        ax.set_ylabel("CARLA y position [m]", fontsize=8)
        ax.axis("equal")
        style_axis(ax)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.03),
        ncol=3,
        fontsize=8,
        frameon=True,
    )
    fig.suptitle(
        f"Trajectory comparison at {speed} m/s",
        fontsize=10,
        y=1.10,
    )
    fig.tight_layout()

    save_figure(
        fig,
        output_dir / f"trajectory_comparison_{speed}ms.png",
        dpi,
    )


def make_cte_figure(
    data_dir: Path,
    speed: int,
    output_dir: Path,
    dpi: int,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.0))

    for ax, (path_name, path_title) in zip(axes, PATHS.items()):
        pp = load_run(csv_path(data_dir, path_name, "pure_pursuit", speed))
        stanley = load_run(csv_path(data_dir, path_name, "stanley", speed))

        ax.plot(
            pp["time"],
            pp["cross_track_error"].abs(),
            linewidth=1.25,
            label="Pure Pursuit",
        )
        ax.plot(
            stanley["time"],
            stanley["cross_track_error"].abs(),
            linewidth=1.25,
            label="Stanley",
        )

        ax.set_title(path_title, fontsize=9)
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.set_ylabel("Absolute CTE [m]", fontsize=8)
        ax.set_ylim(bottom=0.0)
        style_axis(ax)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.03),
        ncol=2,
        fontsize=8,
        frameon=True,
    )
    fig.suptitle(
        f"Cross-track error at {speed} m/s",
        fontsize=10,
        y=1.10,
    )
    fig.tight_layout()

    save_figure(
        fig,
        output_dir / f"cross_track_error_{speed}ms.png",
        dpi,
    )


def steering_rate_deg_s(df: pd.DataFrame) -> np.ndarray:
    time = df["time"].to_numpy(dtype=float)
    steering = df["steering_angle_rad"].to_numpy(dtype=float)

    if len(time) < 2:
        return np.asarray([0.0])

    dt = np.diff(time)
    dsteer = np.diff(steering)
    valid = dt > 0

    if not np.any(valid):
        return np.asarray([0.0])

    return np.degrees(dsteer[valid] / dt[valid])


def summarize_run(
    df: pd.DataFrame,
    path_name: str,
    controller: str,
    speed: int,
) -> dict[str, float | int | str]:
    cte = df["cross_track_error"].to_numpy(dtype=float)
    heading = df["heading_error"].to_numpy(dtype=float)
    steer_rate = steering_rate_deg_s(df)

    result: dict[str, float | int | str] = {
        "path": path_name,
        "speed_mps": speed,
        "controller": CONTROLLERS[controller],
        "mean_abs_cte_m": float(np.mean(np.abs(cte))),
        "rms_cte_m": float(np.sqrt(np.mean(cte**2))),
        "max_abs_cte_m": float(np.max(np.abs(cte))),
        "mean_abs_heading_error_deg": float(
            np.mean(np.degrees(np.abs(heading)))
        ),
        "max_abs_heading_error_deg": float(
            np.max(np.degrees(np.abs(heading)))
        ),
        "mean_abs_steering_rate_deg_s": float(
            np.mean(np.abs(steer_rate))
        ),
        "mean_speed_mps": float(df["speed"].mean()),
        "duration_s": float(df["time"].iloc[-1] - df["time"].iloc[0]),
    }

    if "termination_reason" in df.columns:
        result["termination_reason"] = str(df["termination_reason"].iloc[-1])

    return result


def build_summary(
    data_dir: Path,
    speeds: list[int],
) -> pd.DataFrame:
    rows = []

    for path_name in PATHS:
        for speed in speeds:
            for controller in CONTROLLERS:
                df = load_run(
                    csv_path(data_dir, path_name, controller, speed)
                )
                rows.append(
                    summarize_run(df, path_name, controller, speed)
                )

    return pd.DataFrame(rows)


def make_metric_summary_figure(
    summary: pd.DataFrame,
    output_dir: Path,
    dpi: int,
) -> None:
    """
    One compact figure with two panels:
      (a) RMS cross-track error across all speeds
      (b) mean absolute steering rate across all speeds
    """
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.15))

    x_labels = []
    for path_name in PATHS:
        short_name = "Curvy" if path_name == "smooth_curvy" else "Sharp"
        for speed in sorted(summary["speed_mps"].unique()):
            x_labels.append(f"{short_name}\n{speed} m/s")

    x = np.arange(len(x_labels))
    width = 0.36

    metrics = [
        (
            "rms_cte_m",
            "RMS cross-track error [m]",
            "Tracking accuracy",
        ),
        (
            "mean_abs_steering_rate_deg_s",
            "Mean steering rate [deg/s]",
            "Steering activity",
        ),
    ]

    for ax, (metric, ylabel, title) in zip(axes, metrics):
        pp_values = []
        stanley_values = []

        for path_name in PATHS:
            for speed in sorted(summary["speed_mps"].unique()):
                subset = summary[
                    (summary["path"] == path_name)
                    & (summary["speed_mps"] == speed)
                ]

                pp_values.append(
                    float(
                        subset[
                            subset["controller"] == "Pure Pursuit"
                        ][metric].iloc[0]
                    )
                )
                stanley_values.append(
                    float(
                        subset[
                            subset["controller"] == "Stanley"
                        ][metric].iloc[0]
                    )
                )

        ax.bar(
            x - width / 2,
            pp_values,
            width,
            label="Pure Pursuit",
        )
        ax.bar(
            x + width / 2,
            stanley_values,
            width,
            label="Stanley",
        )

        ax.set_title(title, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_xticks(x)
        ax.set_xticklabels(x_labels, fontsize=7)
        ax.set_ylim(bottom=0.0)
        style_axis(ax)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=2,
        fontsize=8,
        frameon=True,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.92))

    save_figure(fig, output_dir / "metric_summary.png", dpi)


def main() -> int:
    args = parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    make_trajectory_figure(
        data_dir=args.data_dir,
        speed=args.speed,
        output_dir=args.output_dir,
        dpi=args.dpi,
    )

    make_cte_figure(
        data_dir=args.data_dir,
        speed=args.speed,
        output_dir=args.output_dir,
        dpi=args.dpi,
    )

    summary = build_summary(args.data_dir, args.speeds)
    summary_path = args.output_dir / "all_summary_metrics.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Saved: {summary_path}")

    make_metric_summary_figure(
        summary=summary,
        output_dir=args.output_dir,
        dpi=args.dpi,
    )

    print("\nSummary metrics:")
    print(summary.to_string(index=False))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
