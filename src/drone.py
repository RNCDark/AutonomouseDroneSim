
# src/drone.py

from enum import Enum

from src.pathfinding import AStar
from src.sensors import VirtualSensor


class DroneState(Enum):
    """Possible operational states of the autonomous drone."""

    READY = "READY"
    MOVING = "MOVING"
    PAUSED = "PAUSED"
    REROUTING = "REROUTING"
    ARRIVED = "ARRIVED"


class Drone:
    """
    Controls autonomous drone navigation within the environment.

    The drone uses A* to plan routes and a virtual sensor
    to detect obstacles before moving.
    """

    def __init__(self, environment):
        self.environment = environment

        # The drone begins at the environment's start position.
        self.position = environment.start
        self.goal = environment.goal

        # Navigation components.
        self.pathfinder = AStar(environment)
        self.sensor = VirtualSensor(environment)

        # The planned route contains grid positions.
        self.path = []

        # Operational state and performance counters.
        self.state = DroneState.READY
        self.steps_taken = 0
        self.reroute_count = 0
        self.pause_count = 0

        # Record every position visited by the drone.
        self.travel_history = [self.position]

    def plan_path(self):
        """
        Calculate a new route from the drone's current position.

        This method is used for both initial navigation
        and future route recalculations.
        """

        self.path = self.pathfinder.find_path(
            start=self.position,
            goal=self.goal
        )

        # A valid route includes the drone's current position.
        if self.path:
            self.state = (
                DroneState.ARRIVED
                if self.position == self.goal
                else DroneState.MOVING
            )
            return True

        # If A* cannot find a route, pause safely.
        self.state = DroneState.PAUSED
        self.pause_count += 1

        return False

    def start(self):
        """Begin autonomous navigation."""

        return self.plan_path()

    def update(self, dynamic_obstacles=None):
        """
        Advance the drone by at most one grid cell.

        Before moving, the drone scans for nearby obstacles
        and checks whether its planned route is obstructed.
        """

        if dynamic_obstacles is None:
            dynamic_obstacles = set()

        # Stop updating once the destination is reached.
        if self.state == DroneState.ARRIVED:
            return

        # If no route exists, attempt to plan one.
        if not self.path:
            self.plan_path()
            return

        # Scan the surrounding environment.
        detected_obstacles = self.sensor.scan(
            self.position,
            dynamic_obstacles
        )

        # The first path element is the current position.
        # The remaining elements represent upcoming movements.
        upcoming_path = self.path[1:]

        # Only consider upcoming cells inside sensor range.
        visible_path = [
            cell for cell in upcoming_path
            if (
                abs(cell[0] - self.position[0])
                + abs(cell[1] - self.position[1])
                <= self.sensor.detection_range
            )
        ]

        # Check whether the sensor detected a blocked route.
        if self.sensor.is_path_blocked(
            visible_path,
            detected_obstacles
        ):
            self.state = DroneState.REROUTING
            self.reroute_count += 1

            # Recalculate the route around known obstacles.
            # Dynamic obstacle integration will be completed
            # in the next development phase.
            self.plan_path()
            return

        # If the route contains only the current position,
        # check whether the drone has reached its goal.
        if len(self.path) <= 1:
            if self.position == self.goal:
                self.state = DroneState.ARRIVED
            else:
                self.plan_path()
            return

        # Select the next position along the planned route.
        next_position = self.path[1]

        # Never enter an occupied cell.
        if (
            not self.environment.is_walkable(next_position)
            or next_position in dynamic_obstacles
        ):
            self.state = DroneState.PAUSED
            self.pause_count += 1
            return

        # Move the drone forward by one grid cell.
        self.position = next_position
        self.steps_taken += 1

        # Record movement for future performance analysis.
        self.travel_history.append(self.position)

        # Remove the previous position from the route.
        self.path.pop(0)

        # Determine whether the destination has been reached.
        if self.position == self.goal:
            self.state = DroneState.ARRIVED
        else:
            self.state = DroneState.MOVING
