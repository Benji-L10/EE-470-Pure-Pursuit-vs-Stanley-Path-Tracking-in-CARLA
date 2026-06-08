import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(csv_path):
    df = pd.read_csv(csv_path)

    required_columns = [
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
    ]

    missing = [column for column in required_columns if column not in df.columns]

    if missing:
        raise ValueError(
            f"{csv_path} is missing the following columns: {missing}"
        )

    return df


def save_plot(output_path):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"Saved {output_path}")


def plot_trajectory(pp_df, stanley_df, output_path):
    plt.figure(figsize=(9, 6))

    # Remove repeated reference points before plotting.
    reference_df = pp_df[
        ["reference_x", "reference_y", "closest_path_index"]
    ].drop_duplicates(subset="closest_path_index")

    plt.plot(
        reference_df["reference_x"],
        reference_df["reference_y"],
        "--",
        linewidth=2,
        label="Reference path",
    )

    plt.plot(
        pp_df["x"],
        pp_df["y"],
        linewidth=2,
        label="Pure Pursuit",
    )

    plt.plot(
        stanley_df["x"],
        stanley_df["y"],
        linewidth=2,
        label="Stanley",
    )

    plt.xlabel("CARLA x position [m]")
    plt.ylabel("CARLA y position [m]")
    plt.title("Trajectory Comparison")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    save_plot(output_path)


def plot_cross_track_error(pp_df, stanley_df, output_path):
    plt.figure(figsize=(9, 5))

    plt.plot(
        pp_df["time"],
        pp_df["cross_track_error"].abs(),
        label="Pure Pursuit",
    )

    plt.plot(
        stanley_df["time"],
        stanley_df["cross_track_error"].abs(),
        label="Stanley",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Absolute cross-track error [m]")
    plt.title("Cross-Track Error")
    plt.grid(True)
    plt.legend()

    save_plot(output_path)


def plot_heading_error(pp_df, stanley_df, output_path):
    plt.figure(figsize=(9, 5))

    plt.plot(
        pp_df["time"],
        np.degrees(pp_df["heading_error"].abs()),
        label="Pure Pursuit",
    )

    plt.plot(
        stanley_df["time"],
        np.degrees(stanley_df["heading_error"].abs()),
        label="Stanley",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Absolute heading error [deg]")
    plt.title("Heading Error")
    plt.grid(True)
    plt.legend()

    save_plot(output_path)


def plot_steering(pp_df, stanley_df, output_path):
    plt.figure(figsize=(9, 5))

    plt.plot(
        pp_df["time"],
        np.degrees(pp_df["steering_angle_rad"]),
        label="Pure Pursuit",
    )

    plt.plot(
        stanley_df["time"],
        np.degrees(stanley_df["steering_angle_rad"]),
        label="Stanley",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Steering angle [deg]")
    plt.title("Steering Command")
    plt.grid(True)
    plt.legend()

    save_plot(output_path)


def plot_speed(pp_df, stanley_df, output_path):
    plt.figure(figsize=(9, 5))

    plt.plot(
        pp_df["time"],
        pp_df["target_speed"],
        "--",
        linewidth=2,
        label="Target speed",
    )

    plt.plot(
        pp_df["time"],
        pp_df["speed"],
        label="Pure Pursuit",
    )

    plt.plot(
        stanley_df["time"],
        stanley_df["speed"],
        label="Stanley",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed Tracking")
    plt.grid(True)
    plt.legend()

    save_plot(output_path)


def calculate_summary(df, controller_name):
    steering = df["steering_angle_rad"].to_numpy()
    time = df["time"].to_numpy()

    if len(df) > 1:
        dt = np.diff(time)
        steering_rate = np.diff(steering) / dt
    else:
        steering_rate = np.array([0.0])

    return {
        "controller": controller_name,
        "mean_abs_cte_m": df["cross_track_error"].abs().mean(),
        "rms_cte_m": np.sqrt(
            np.mean(df["cross_track_error"] ** 2)
        ),
        "max_abs_cte_m": df["cross_track_error"].abs().max(),
        "mean_abs_heading_error_deg": np.degrees(
            df["heading_error"].abs()
        ).mean(),
        "max_abs_heading_error_deg": np.degrees(
            df["heading_error"].abs()
        ).max(),
        "mean_abs_steering_rate_deg_s": np.degrees(
            np.abs(steering_rate)
        ).mean(),
        "mean_speed_mps": df["speed"].mean(),
    }


def save_summary(pp_df, stanley_df, output_path):
    summary = pd.DataFrame(
        [
            calculate_summary(pp_df, "Pure Pursuit"),
            calculate_summary(stanley_df, "Stanley"),
        ]
    )

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    summary.to_csv(output_path, index=False)

    print("\nSummary metrics:")
    print(summary.to_string(index=False))
    print(f"\nSaved {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot Pure Pursuit and Stanley CARLA results."
    )

    parser.add_argument(
        "--pure-pursuit",
        required=True,
        help="Path to the Pure Pursuit CSV file.",
    )

    parser.add_argument(
        "--stanley",
        required=True,
        help="Path to the Stanley CSV file.",
    )

    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory where plots will be saved.",
    )

    args = parser.parse_args()

    pp_df = load_csv(args.pure_pursuit)
    stanley_df = load_csv(args.stanley)

    plot_trajectory(
        pp_df,
        stanley_df,
        os.path.join(args.output_dir, "trajectory_comparison.png"),
    )

    plot_cross_track_error(
        pp_df,
        stanley_df,
        os.path.join(args.output_dir, "cross_track_error.png"),
    )

    plot_heading_error(
        pp_df,
        stanley_df,
        os.path.join(args.output_dir, "heading_error.png"),
    )

    plot_steering(
        pp_df,
        stanley_df,
        os.path.join(args.output_dir, "steering_command.png"),
    )

    plot_speed(
        pp_df,
        stanley_df,
        os.path.join(args.output_dir, "speed_tracking.png"),
    )

    save_summary(
        pp_df,
        stanley_df,
        os.path.join(args.output_dir, "summary_metrics.csv"),
    )


if __name__ == "__main__":
    main()