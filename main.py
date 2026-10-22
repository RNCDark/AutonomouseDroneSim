
# main.py

from src.environment import Environment
from src.pathfinding import AStar


def main():
    environment = Environment()
    pathfinder = AStar(environment)

    path = pathfinder.find_path()

    print("Autonomous Drone Simulation")
    print("-" * 40)

    print(f"Grid size: {environment.width} x {environment.height}")
    print(f"Static obstacles: {environment.get_obstacle_count()}")

    if path:
        print("\nA* Path Found!")
        print(f"Path length: {len(path) - 1} movements")
        print(f"Nodes explored: {pathfinder.nodes_explored}")

        print("\nCalculated path:")
        print(path)

        print("\nEnvironment with path:")

        path_cells = set(path)

        for row in range(environment.height):
            symbols = []

            for col in range(environment.width):
                position = (row, col)

                if position == environment.start:
                    symbol = "S"
                elif position == environment.goal:
                    symbol = "G"
                elif position in path_cells:
                    symbol = "*"
                elif environment.is_obstacle(position):
                    symbol = "#"
                else:
                    symbol = "."

                symbols.append(symbol)

            print(" ".join(symbols))

    else:
        print("\nNo valid path found between start and goal.")

if __name__ == "__main__":
    main()
