import numpy as np


class StanleyController:
    def __init__(self, wheelbase, k=0.8, ks=0.2, max_steer=np.radians(30)):
        self.L = wheelbase
        self.k = k
        self.ks = ks
        self.max_steer = max_steer

    def compute_control(self, vehicle_state, path):
        """
        vehicle_state: dict with x, y, yaw, speed
        path: array of path points [[x, y], ...]

        returns:
            steering_angle_rad
        """
        x = vehicle_state["x"]
        y = vehicle_state["y"]
        yaw = vehicle_state["yaw"]
        speed = vehicle_state["speed"]

        front_x = x + self.L * np.cos(yaw)
        front_y = y + self.L * np.sin(yaw)

        closest_idx = self._find_closest_point(front_x, front_y, path)
        closest_point = path[closest_idx]

        path_yaw = self._compute_path_yaw(path, closest_idx)

        heading_error = self._wrap_to_pi(path_yaw - yaw)

        dx = closest_point[0] - front_x
        dy = closest_point[1] - front_y

        # Signed crosstrack error using path normal direction
        normal_x = -np.sin(path_yaw)
        normal_y = np.cos(path_yaw)
        cross_track_error = dx * normal_x + dy * normal_y

        cte_term = np.arctan2(self.k * cross_track_error, speed + self.ks)

        delta = heading_error + cte_term
        delta = self._wrap_to_pi(delta)
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        return delta

    @staticmethod
    def _find_closest_point(x, y, path):
        distances = np.linalg.norm(path - np.array([x, y]), axis=1)
        return int(np.argmin(distances))

    @staticmethod
    def _compute_path_yaw(path, idx):
        if idx < len(path) - 1:
            dx = path[idx + 1, 0] - path[idx, 0]
            dy = path[idx + 1, 1] - path[idx, 1]
        else:
            dx = path[idx, 0] - path[idx - 1, 0]
            dy = path[idx, 1] - path[idx - 1, 1]

        return np.arctan2(dy, dx)

    @staticmethod
    def _wrap_to_pi(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi