import numpy as np


class PurePursuitController:
    def __init__(
        self,
        wheelbase,
        lookahead_gain=0.5,
        min_lookahead=3.0,
        max_steer=np.radians(30),
        search_window=100,
    ):
        self.L = wheelbase
        self.Kv = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_steer = max_steer
        self.search_window = search_window

        # Stores progress along the path so the controller does not jump backward.
        self.previous_closest_idx = 0

    def compute_lookahead_distance(self, speed):
        return max(self.min_lookahead, self.Kv * speed)

    def compute_control(self, vehicle_state, path):
        """
        Parameters
        ----------
        vehicle_state : dict
            Must contain x, y, yaw, and speed.
        path : np.ndarray
            Array with shape (N, 2), containing [x, y] path points.

        Returns
        -------
        float
            Steering angle in radians.
        """
        path = np.asarray(path, dtype=float)

        if path.ndim != 2 or path.shape[1] != 2:
            raise ValueError("Path must be a NumPy array with shape (N, 2).")

        if len(path) < 2:
            raise ValueError("Path must contain at least two points.")

        x = float(vehicle_state["x"])
        y = float(vehicle_state["y"])
        yaw = float(vehicle_state["yaw"])
        speed = float(vehicle_state["speed"])

        lookahead_distance = self.compute_lookahead_distance(speed)

        closest_idx = self._find_closest_forward_index(x, y, path)
        goal_idx = self._find_lookahead_index(
            path,
            closest_idx,
            lookahead_distance,
        )

        goal = path[goal_idx]

        dx = goal[0] - x
        dy = goal[1] - y

        angle_to_goal = np.arctan2(dy, dx)
        alpha = self._wrap_to_pi(angle_to_goal - yaw)

        # Use the actual distance to the selected goal point. This is more
        # accurate when waypoint spacing does not land exactly on Ld.
        actual_goal_distance = np.hypot(dx, dy)
        actual_goal_distance = max(actual_goal_distance, 1e-6)

        delta = np.arctan2(
            2.0 * self.L * np.sin(alpha),
            actual_goal_distance,
        )

        return float(np.clip(delta, -self.max_steer, self.max_steer))

    def _find_closest_forward_index(self, x, y, path):
        """
        Find the closest path point near the previous closest point.

        Searching only a forward local window prevents the controller from
        jumping backward or selecting another nearby section of the route.
        """
        start_idx = max(0, self.previous_closest_idx - 2)
        end_idx = min(len(path), self.previous_closest_idx + self.search_window)

        local_path = path[start_idx:end_idx]
        vehicle_position = np.array([x, y])

        local_distances = np.linalg.norm(
            local_path - vehicle_position,
            axis=1,
        )

        closest_idx = start_idx + int(np.argmin(local_distances))

        # Prevent path progress from moving significantly backward.
        self.previous_closest_idx = max(
            self.previous_closest_idx,
            closest_idx,
        )

        return self.previous_closest_idx

    @staticmethod
    def _find_lookahead_index(path, closest_idx, lookahead_distance):
        """
        Move forward along the path until the accumulated arc length reaches Ld.
        """
        accumulated_distance = 0.0
        goal_idx = closest_idx

        for idx in range(closest_idx, len(path) - 1):
            segment_distance = np.linalg.norm(path[idx + 1] - path[idx])
            accumulated_distance += segment_distance
            goal_idx = idx + 1

            if accumulated_distance >= lookahead_distance:
                break

        return goal_idx

    def reset(self):
        """Reset stored path progress before starting a new experiment."""
        self.previous_closest_idx = 0

    @staticmethod
    def _wrap_to_pi(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi