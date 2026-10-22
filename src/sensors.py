
# src/sensors.py

from src.config import SENSOR_RANGE


class VirtualSensor:
    """
    Simulates environmental perception using a grid-based sensor.

    The sensor detects obstacles within a Manhattan-distance
    radius around the drone's current position.
    """

    def __init__(self, environment, detection_range=SENSOR_RANGE):
        self.environment = environment
        self.detection_range = detection_range

    def scan(self, position, dynamic_obstacles=None):
        """
        Return all detected obstacles within sensor range.

        Static obstacles come from the environment grid.
        Dynamic obstacles are supplied separately because
        their positions can change during the simulation.
        """

        if dynamic_obstacles is None:
            dynamic_obstacles = set()

        detected = set()

        row, col = position

        # Examine every cell inside the sensor's bounding square.
        for dr in range(
            -self.detection_range,
            self.detection_range + 1
        ):
            for dc in range(
                -self.detection_range,
                self.detection_range + 1
            ):

                # Ignore cells outside the Manhattan-distance radius.
                if abs(dr) + abs(dc) > self.detection_range:
                    continue

                target = (row + dr, col + dc)

                # Ignore positions outside the environment.
                if not self.environment.is_within_bounds(target):
                    continue

                # Detect either a static or dynamic obstacle.
                if (
                    self.environment.is_obstacle(target)
                    or target in dynamic_obstacles
                ):
                    detected.add(target)

        return detected

    def is_path_blocked(self, path, detected_obstacles):
        """
        Determine whether any upcoming route cell is obstructed.

        This allows the drone to identify a blocked route
        before attempting to move into an occupied cell.
        """

        return any(
            position in detected_obstacles
            for position in path
        )
