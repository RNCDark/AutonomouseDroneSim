
# tests/tests_simulation.py

from src.environment import Environment
from src.drone import Drone, DroneState


def test_dynamic_obstacle_triggers_reroute():
    """
    Verify that the drone detects an unexpected obstacle,
    calculates an alternative route, and reaches its goal.

    A controlled obstacle placement makes this test
    reproducible instead of depending on random movement.
    """

    # Create an empty environment so the only obstruction
    # comes from the dynamic obstacle introduced below.
    environment = Environment(
        width=5,
        height=5,
        obstacle_density=0.0,
        seed=42,
        start=(0, 0),
        goal=(4, 4)
    )

    drone = Drone(environment)

    # Calculate the initial route without dynamic obstacles.
    assert drone.start()

    # Select the next cell on the calculated route.
    # This guarantees the obstacle blocks the planned path.
    blocked_position = drone.path[1]

    # Introduce the obstacle after the route is calculated.
    dynamic_obstacles = {blocked_position}

    # Update once: the drone should detect the obstacle
    # and request a new route without entering the cell.
    drone.update(dynamic_obstacles)

    assert drone.reroute_count == 1
    assert drone.position == environment.start
    assert blocked_position not in drone.path

    # Continue navigation while the obstacle remains fixed.
    for _ in range(30):
        if drone.state == DroneState.ARRIVED:
            break

        drone.update(dynamic_obstacles)

    # Confirm the drone reaches the destination safely.
    assert drone.state == DroneState.ARRIVED
    assert drone.position == environment.goal
    assert blocked_position not in drone.travel_history


def test_dynamic_obstacle_never_causes_collision():
    """
    Verify that the drone never moves into a cell
    occupied by a dynamic obstacle.
    """

    environment = Environment(
        width=5,
        height=5,
        obstacle_density=0.0,
        seed=42,
        start=(0, 0),
        goal=(4, 4)
    )

    drone = Drone(environment)
    assert drone.start()

    dynamic_obstacles = {drone.path[1]}

    for _ in range(30):
        if drone.state == DroneState.ARRIVED:
            break

        drone.update(dynamic_obstacles)

        # Check the drone's position after every update.
        assert drone.position not in dynamic_obstacles

    assert drone.state == DroneState.ARRIVED


def test_drone_recovers_after_temporary_blockage():
    """
    Verify that the drone pauses when all routes are blocked
    and resumes navigation once the obstacles are removed.

    This tests recovery behavior rather than ordinary
    obstacle avoidance.
    """

    # Create a small, obstacle-free environment.
    # The drone must cross column 1 to reach the goal.
    environment = Environment(
        width=3,
        height=3,
        obstacle_density=0.0,
        seed=42,
        start=(1, 0),
        goal=(1, 2)
    )

    drone = Drone(environment)

    # Verify that a valid initial route exists.
    assert drone.start()

    # Block the entire middle column.
    # No path from the left side to the right side exists.
    dynamic_obstacles = {
        (0, 1),
        (1, 1),
        (2, 1)
    }

    # The drone should detect the blocked route
    # and attempt to calculate an alternative.
    drone.update(dynamic_obstacles)

    # Because no alternative exists, it must pause.
    assert drone.state == DroneState.PAUSED

    # Confirm the drone did not move into an obstacle.
    assert drone.position == (1, 0)
    assert drone.position not in dynamic_obstacles

    # Confirm that a rerouting attempt occurred.
    assert drone.reroute_count == 1
    assert drone.pause_count == 1

    # Simulate moving obstacles leaving the corridor.
    dynamic_obstacles.clear()

    # Allow enough updates for route recovery
    # and movement to the destination.
    for _ in range(10):

        if drone.state == DroneState.ARRIVED:
            break

        drone.update(dynamic_obstacles)

        # Verify collision avoidance after every update.
        assert drone.position not in dynamic_obstacles

    # The drone should successfully recover and arrive.
    assert drone.state == DroneState.ARRIVED
    assert drone.position == environment.goal

    # Only two movements are required once the
    # middle column becomes available again.
    assert drone.steps_taken == 2

