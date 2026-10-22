
# src/environment.py

import numpy as np

from src.config import (
    GRID_WIDTH,
    GRID_HEIGHT,
    OBSTACLE_DENSITY,
    RANDOM_SEED,
    START_POSITION,
    GOAL_POSITION,
)


class Environment:
    """Represents the drone's 2D navigation environment."""

    def __init__(
        self,
        width=GRID_WIDTH,
        height=GRID_HEIGHT,
        obstacle_density=OBSTACLE_DENSITY,
        seed=RANDOM_SEED,
        start=START_POSITION,
        goal=GOAL_POSITION,
    ):
        self.width = width
        self.height = height
        self.obstacle_density = obstacle_density
        self.start = start
        self.goal = goal

        self.rng = np.random.default_rng(seed)

        # 0 = Free cell, 1 = Static obstacle
        self.grid = np.zeros(
            (self.height, self.width),
            dtype=np.int8
        )

        self.generate_obstacles()

    def generate_obstacles(self):
        """Randomly populate the environment with obstacles."""

        self.grid.fill(0)

        obstacle_mask = (
            self.rng.random((self.height, self.width))
            < self.obstacle_density
        )

        self.grid[obstacle_mask] = 1

        # Start and destination must remain accessible
        self.grid[self.start] = 0
        self.grid[self.goal] = 0

    def is_within_bounds(self, position):
        """Check whether a position is inside the grid."""

        row, col = position

        return (
            0 <= row < self.height
            and 0 <= col < self.width
        )

    def is_obstacle(self, position):
        """Check whether a cell contains a static obstacle."""

        if not self.is_within_bounds(position):
            return True

        row, col = position
        return self.grid[row, col] == 1

    def is_walkable(self, position):
        """Return True if a cell is valid and unobstructed."""

        return (
            self.is_within_bounds(position)
            and not self.is_obstacle(position)
        )

    def get_neighbors(self, position):
        """Return valid neighbors using four-direction movement."""

        row, col = position

        directions = [
            (-1, 0),  # Up
            (1, 0),   # Down
            (0, -1),  # Left
            (0, 1),   # Right
        ]

        neighbors = []

        for dr, dc in directions:
            neighbor = (row + dr, col + dc)

            if self.is_walkable(neighbor):
                neighbors.append(neighbor)

        return neighbors

    def get_obstacle_count(self):
        """Return the total number of static obstacles."""

        return int(np.sum(self.grid))

    def display_grid(self):
        """Print an ASCII representation of the environment."""

        for row in range(self.height):
            line = []

            for col in range(self.width):
                position = (row, col)

                if position == self.start:
                    symbol = "S"
                elif position == self.goal:
                    symbol = "G"
                elif self.is_obstacle(position):
                    symbol = "#"
                else:
                    symbol = "."

                line.append(symbol)

            print(" ".join(line))
