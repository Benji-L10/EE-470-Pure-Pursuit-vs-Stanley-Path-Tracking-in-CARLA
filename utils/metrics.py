import numpy as np


def compute_cross_track_error(vehicle_xy, path):
    vehicle_xy = np.asarray(vehicle_xy)
    distances = np.linalg.norm(path - vehicle_xy, axis=1)
    return float(np.min(distances))


def compute_heading_error(vehicle_yaw, path, closest_idx):
    if closest_idx < len(path) - 1:
        dx = path[closest_idx + 1, 0] - path[closest_idx, 0]
        dy = path[closest_idx + 1, 1] - path[closest_idx, 1]
    else:
        dx = path[closest_idx, 0] - path[closest_idx - 1, 0]
        dy = path[closest_idx, 1] - path[closest_idx - 1, 1]

    path_yaw = np.arctan2(dy, dx)
    return wrap_to_pi(path_yaw - vehicle_yaw)


def summarize_metrics(df):
    summary = {}

    summary["mean_cte"] = df["cross_track_error"].abs().mean()
    summary["max_cte"] = df["cross_track_error"].abs().max()
    summary["rms_cte"] = np.sqrt(np.mean(df["cross_track_error"] ** 2))

    summary["mean_heading_error"] = df["heading_error"].abs().mean()
    summary["max_heading_error"] = df["heading_error"].abs().max()

    steering_diff = df["steering"].diff().dropna()
    summary["mean_steering_change"] = steering_diff.abs().mean()
    summary["rms_steering_change"] = np.sqrt(np.mean(steering_diff ** 2))

    return summary


def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi