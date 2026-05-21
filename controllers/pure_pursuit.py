import numpy as np


class PurePursuitController:
    def __init__(self, wheelbase, lookahead_gain=0.5, min_lookahead=3.0, max_steer=np.radians(30)):
        self.L = wheelbase
        self.Kv = lookahead_gain
        self.min_lookahead = min_lookahead
        self.max_steer = max_steer

    def compute_lookahead_distance(self, speed):
        return max(self.min_lookahead, self.Kv * speed)

    def compute_control(self, vehicle_state, path):
        """
        vehicle_state: dict with x, y, yaw, speed
        path: list or array of path points [[x, y], ...]

        returns:
            steering_angle_rad
        """
        x = vehicle_state["x"]
        y = vehicle_state["y"]
        yaw = vehicle_state["yaw"]
        speed = vehicle_state["speed"]

        Ld = self.compute_lookahead_distance(speed)

        goal = self._find_lookahead_point(x, y, path, Ld)

        dx = goal[0] - x
        dy = goal[1] - y

        angle_to_goal = np.arctan2(dy, dx)
        alpha = self._wrap_to_pi(angle_to_goal - yaw)

        delta = np.arctan2(2.0 * self.L * np.sin(alpha), Ld)
        delta = np.clip(delta, -self.max_steer, self.max_steer)

        return delta

    def _find_lookahead_point(self, x, y, path, Ld):
        distances = np.linalg.norm(path - np.array([x, y]), axis=1)

        for i in range(len(path)):
            if distances[i] >= Ld:
                return path[i]

        return path[-1]

    @staticmethod
    def _wrap_to_pi(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi