
# main.py
from src.simulation import Simulation


def main():
    """
    Entry point for the autonomous drone simulation.

    Simulation manages the environment, navigation logic,
    dynamic obstacles, and real-time visualization.
    """

    simulation = Simulation()
    simulation.run()

if __name__ == "__main__":
    main()
