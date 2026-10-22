
# src/environment.py

import numpy as np

from src.config import (
    GRID_WIDTH,
    GRID_HEIGHT,
    OBSTACLE_DENSITY,
    NUM_DYNAMIC_OBSTACLES,
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
        self.dynamic_obstacles = set()
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
    
    def initialize_dynamic_obstacles(
        self,
        count=NUM_DYNAMIC_OBSTACLES,
        excluded_positions=None
    ):
        """
        Place moving obstacles in valid, unoccupied grid cells.

        Excluded positions prevent obstacles from spawning
        on the drone, destination, or other protected cells.
        """
        excluded = set(excluded_positions or [])
        excluded.update({self.start, self.goal})

        available = [
            (row, col)
            for row in range(self.height)
            for col in range(self.width)
            if (
                self.is_walkable((row, col))
                and (row, col) not in excluded
            )
        ]

        if count > len(available):
            raise ValueError(
                "Not enough available cells for dynamic obstacles."
            )

        self.dynamic_obstacles = set()

        # Sample unique cells without replacement.
        if count > 0:
            indices = self.rng.choice(
                len(available),
                size=count,
                replace=False
            )

            for index in indices:
                self.dynamic_obstacles.add(
                    available[int(index)]
                )

        return self.dynamic_obstacles


    def move_dynamic_obstacles(self, drone_position):
        """
        Move each dynamic obstacle by at most one grid cell.

        Obstacles cannot move into static obstacles,
        the destination, the drone, or each other.
        """

        # Keep track of occupied cells to prevent overlap.
        occupied = set(self.dynamic_obstacles)
        new_positions = set()

        # Sort for reproducible update ordering.
        for position in sorted(self.dynamic_obstacles):

            # Free the current position for this obstacle.
            occupied.remove(position)

            row, col = position

            candidates = [
                position,          # Remain stationary
                (row - 1, col),    # Up
                (row + 1, col),    # Down
                (row, col - 1),    # Left
                (row, col + 1),    # Right
            ]

            valid_positions = [
                candidate
                for candidate in candidates
                if (
                    self.is_walkable(candidate)
                    and candidate != drone_position
                    and candidate != self.goal
                    and candidate != self.start
                    and candidate not in occupied
                    and candidate not in new_positions
                )
            ]

            # Remaining stationary is always a fallback
            # because this obstacle's original cell is free.
            selected = valid_positions[
                int(self.rng.integers(len(valid_positions)))
            ]

            new_positions.add(selected)

        self.dynamic_obstacles = new_positions

        return self.dynamic_obstacles

