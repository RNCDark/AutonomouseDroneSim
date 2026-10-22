
# main.py

from src.environment import Environment
from src.drone import Drone, DroneState


def main():
    environment = Environment()
    drone = Drone(environment)

    # Generate five moving obstacles without placing
    # them directly on the initial planned route.
    drone.start()

    environment.initialize_dynamic_obstacles(
        excluded_positions=set(drone.path)
    )

    print("Autonomous Drone Simulation")
    print("-" * 40)

    print(f"Start: {drone.position}")
    print(f"Goal: {drone.goal}")
    print(f"Static obstacles: {environment.get_obstacle_count()}")
    print(
        f"Dynamic obstacles: "
        f"{len(environment.dynamic_obstacles)}"
    )

    print("\nBeginning dynamic navigation...\n")

    max_updates = 500

    for update_number in range(max_updates):

        if drone.state == DroneState.ARRIVED:
            break

        # Update obstacle positions every third simulation step.
        if update_number % 3 == 0:
            environment.move_dynamic_obstacles(
                drone_position=drone.position
            )

        previous_position = drone.position
        previous_reroutes = drone.reroute_count

        # The drone observes the environment and acts.
        drone.update(
            dynamic_obstacles=environment.dynamic_obstacles
        )

        if drone.reroute_count > previous_reroutes:
            print(
                f"Update {update_number:03d}: "
                f"REROUTING at {drone.position}"
            )

        if drone.position != previous_position:
            print(
                f"Step {drone.steps_taken:02d}: "
                f"{previous_position} -> {drone.position}"
            )

    print("\nSimulation Results")
    print("-" * 40)

    print(f"Final position: {drone.position}")
    print(f"Final state: {drone.state.value}")
    print(f"Steps taken: {drone.steps_taken}")
    print(f"Reroutes: {drone.reroute_count}")
    print(f"Pauses: {drone.pause_count}")

if __name__ == "__main__":
    main()
