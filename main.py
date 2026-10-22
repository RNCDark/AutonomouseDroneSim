
# main.py

from src.environment import Environment


def main():
    environment = Environment()

    print("Autonomous Drone Simulation")
    print("-" * 40)

    print(f"Grid size: {environment.width} x {environment.height}")
    print(f"Start: {environment.start}")
    print(f"Goal: {environment.goal}")
    print(f"Static obstacles: {environment.get_obstacle_count()}")

    print("\nGenerated Environment:\n")
    environment.display_grid()

    print("\nStart walkable:", environment.is_walkable(environment.start))
    print("Goal walkable:", environment.is_walkable(environment.goal))

    print(
        "Start neighbors:",
        environment.get_neighbors(environment.start)
    )


if __name__ == "__main__":
    main()
