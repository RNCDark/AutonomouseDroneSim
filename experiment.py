
# experiments.py

import csv
import os

from src.config import (
    GRID_WIDTH,
    GRID_HEIGHT,
    OBSTACLE_DENSITY,
    START_POSITION,
    GOAL_POSITION,
    DRONE_SPEED,
    DYNAMIC_OBSTACLE_INTERVAL,
)

from src.environment import Environment
from src.drone import Drone, DroneState
from src.metrics import SimulationMetrics


# Use the same seeds for both scenarios.
# This ensures each static/dynamic pair uses the same map.
SEEDS = [42, 43, 44, 45, 46, 47, 48, 49, 50, 51]

# Maximum number of drone updates per experiment.
MAX_UPDATES = 500


def run_trial(seed, scenario):
    """
    Run one autonomous navigation experiment.

    The scenario determines whether moving obstacles
    are included. Rendering is disabled so experiments
    can execute quickly and consistently.
    """

    environment = Environment(
        width=GRID_WIDTH,
        height=GRID_HEIGHT,
        obstacle_density=OBSTACLE_DENSITY,
        seed=seed,
        start=START_POSITION,
        goal=GOAL_POSITION
    )

    drone = Drone(environment)

    # Record the optimal route before navigation begins.
    metrics = SimulationMetrics(environment, drone)

    # Begin navigation using the A* pathfinder.
    drone.start()

    # Dynamic scenarios receive moving obstacles.
    # Static scenarios leave the dynamic set empty.
    if scenario == "dynamic":
        environment.initialize_dynamic_obstacles(
            excluded_positions=set(drone.path)
        )

    # Simulated time advances by one drone update interval.
    # This is not actual wall-clock execution time.
    drone_interval = 1.0 / DRONE_SPEED
    elapsed_time = 0.0
    obstacle_timer = 0.0

    # Count updates separately from successful movements.
    updates = 0

    for _ in range(MAX_UPDATES):

        if drone.state == DroneState.ARRIVED:
            break

        elapsed_time += drone_interval
        obstacle_timer += drone_interval
        updates += 1

        # Match the visual simulation's obstacle interval.
        if scenario == "dynamic":

            while obstacle_timer >= DYNAMIC_OBSTACLE_INTERVAL:

                environment.move_dynamic_obstacles(
                    drone_position=drone.position
                )

                obstacle_timer -= DYNAMIC_OBSTACLE_INTERVAL

        # Allow the drone to detect obstacles and move.
        drone.update(
            dynamic_obstacles=environment.dynamic_obstacles
        )

        # Stop once the drone reaches the destination.
        if drone.state == DroneState.ARRIVED:
            break

    # Calculate all navigation performance measurements.
    results = metrics.calculate(
        elapsed_time=elapsed_time,
        scenario=scenario
    )

    # Add experiment identifiers and diagnostic fields.
    results["seed"] = seed
    results["updates"] = updates

    return results


def find_reachable_seeds(target_count=10, starting_seed=42):
    """
    Find random seeds that produce navigable static maps.

    A seed is accepted only when A* can calculate
    a complete route from the start to the goal.

    Both static and dynamic experiments will use
    the same accepted seeds for fair comparison.
    """

    accepted_seeds = []
    seed = starting_seed

    while len(accepted_seeds) < target_count:

        # Generate a candidate environment.
        environment = Environment(
            width=GRID_WIDTH,
            height=GRID_HEIGHT,
            obstacle_density=OBSTACLE_DENSITY,
            seed=seed,
            start=START_POSITION,
            goal=GOAL_POSITION
        )

        # Check whether the environment is navigable.
        drone = Drone(environment)

        if drone.start():
            accepted_seeds.append(seed)

        # Continue until we have enough valid maps.
        seed += 1

    return accepted_seeds


def main():
    """
    Run paired static and dynamic experiments.

    Results are saved to CSV for later analysis
    and visualization in the IEEE report.
    """

    results = []

    # Select ten environments with an existing static route.
    # This isolates dynamic obstacle behavior from maps
    # that are impossible to navigate from the beginning.
    reachable_seeds = find_reachable_seeds(
        target_count=10,
        starting_seed=42
    )

    print(f"Reachable seeds: {reachable_seeds}")

    for seed in reachable_seeds:

        for scenario in ("static", "dynamic"):

            trial = run_trial(seed, scenario)
            results.append(trial)

            print(
                f"Seed: {seed} | "
                f"Scenario: {scenario:7s} | "
                f"Success: {trial['success']} | "
                f"Steps: {trial['distance_traveled']} | "
                f"Reroutes: {trial['reroutes']} | "
                f"Efficiency: "
                f"{trial['path_efficiency_percent']}"
            )

    # Save all experiments in one CSV.
    os.makedirs("results", exist_ok=True)

    filename = "results/reachable_experiment_results.csv"

    with open(
        filename,
        "w",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=list(results[0].keys())
        )

        writer.writeheader()
        writer.writerows(results)

    print("\nExperiment Results")
    print("-" * 40)
    print(f"Total trials: {len(results)}")
    print(f"Results saved to: {filename}")


if __name__ == "__main__":
    main()
