
# src/metrics.py

import csv
import os

from src.drone import DroneState
from src.pathfinding import AStar


class SimulationMetrics:
    """
    Collects and calculates performance measurements
    for an autonomous drone navigation experiment.

    Separating metrics from navigation logic allows
    us to evaluate different simulation scenarios
    without modifying the drone controller.
    """

    def __init__(self, environment, drone):
        self.environment = environment
        self.drone = drone

        # Calculate the optimal route through the original
        # static environment using a separate A* instance.
        #
        # Dynamic obstacles are excluded because we want
        # a consistent baseline for comparing experiments.
        baseline_pathfinder = AStar(environment)

        baseline_path = baseline_pathfinder.find_path(
            start=environment.start,
            goal=environment.goal
        )

        # A path containing N positions requires N-1 moves.
        # If no route exists, the baseline is unavailable.
        self.optimal_distance = (
            len(baseline_path) - 1
            if baseline_path
            else None
        )

    def calculate(self, elapsed_time, scenario="dynamic"):
        """
        Generate a dictionary of simulation results.

        The returned dictionary can be displayed,
        exported to CSV, or used to generate charts.
        """

        distance_traveled = self.drone.steps_taken

        # Navigation succeeds only if the drone
        # actually reaches its destination.
        success = self.drone.state == DroneState.ARRIVED

        # Path efficiency compares actual movement
        # with the shortest available static route.
        #
        # Example:
        # Optimal route = 38 movements
        # Actual route = 40 movements
        #
        # Efficiency = (38 / 40) * 100 = 95%
        if self.optimal_distance is None:
            path_efficiency = None

        elif distance_traveled == 0:
            path_efficiency = 100.0 if success else None

        else:
            path_efficiency = (
                self.optimal_distance / distance_traveled
            ) * 100

        # Count additional movements beyond the baseline.
        extra_distance = (
            distance_traveled - self.optimal_distance
            if self.optimal_distance is not None
            else None
        )

        return {
            "scenario": scenario,
            "success": success,
            "completion_time_seconds": (
                round(elapsed_time, 3) if success else None
            ),
            "elapsed_time_seconds": round(elapsed_time, 3),
            "optimal_distance": self.optimal_distance,
            "distance_traveled": distance_traveled,
            "extra_distance": extra_distance,
            "path_efficiency_percent": (
                round(path_efficiency, 2)
                if path_efficiency is not None
                else None
            ),
            "reroutes": self.drone.reroute_count,
            "pauses": self.drone.pause_count,
        }

    @staticmethod
    def save_to_csv(results, filename="results/simulation_metrics.csv"):
        """
        Save one or more experiment results to a CSV file.

        Accepts either a single dictionary or a list
        of dictionaries representing multiple trials.
        """

        # Convert a single result into a one-element list.
        if isinstance(results, dict):
            results = [results]

        if not results:
            raise ValueError("No simulation results to export.")

        # Create the results directory if it does not exist.
        directory = os.path.dirname(filename)

        if directory:
            os.makedirs(directory, exist_ok=True)

        # Use the result dictionary keys as column headers.
        fieldnames = list(results[0].keys())

        with open(
            filename,
            mode="w",
            newline="",
            encoding="utf-8"
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fieldnames
            )

            writer.writeheader()
            writer.writerows(results)

        print(f"Metrics saved to: {filename}")
