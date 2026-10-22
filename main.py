
# main.py

from src.environment import Environment
from src.drone import Drone, DroneState


def main():
    environment = Environment()
    drone = Drone(environment)

    print("Autonomous Drone Simulation")
    print("-" * 40)

    print(f"Starting position: {drone.position}")
    print(f"Destination: {drone.goal}")

    if not drone.start():
        print("No valid route available.")
        return

    print(f"Initial route length: {len(drone.path) - 1}")
    print("\nBeginning autonomous navigation...\n")

    # Limit the number of updates to prevent infinite loops.
    max_updates = 500

    for update_number in range(max_updates):

        if drone.state == DroneState.ARRIVED:
            break

        previous_position = drone.position

        drone.update()

        # Display movement only when the position changes.
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
