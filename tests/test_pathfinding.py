
# tests/test_pathfinding.py

import numpy as np
import pytest

from src.environment import Environment
from src.pathfinding import AStar


@pytest.fixture
def environment():
    """
    Create a predictable 5x5 environment for testing.

    We use a smaller grid without randomly generated obstacles
    so each test has a known and reproducible result.
    """
    return Environment(
        width=5,
        height=5,
        obstacle_density=0.0,
        seed=42,
        start=(0, 0),
        goal=(4, 4)
    )


def test_empty_grid_shortest_path(environment):
    """
    Test 1: A* should find the optimal path on an empty grid.

    The shortest distance from (0,0) to (4,4) is 8 moves:
    four downward movements and four rightward movements.
    """
    pathfinder = AStar(environment)
    path = pathfinder.find_path()

    assert path[0] == (0, 0)
    assert path[-1] == (4, 4)
    assert len(path) - 1 == 8


def test_obstacle_avoidance(environment):
    """
    Test 2: A* must avoid blocked cells.

    We construct a vertical wall in column 1.
    The drone must travel around the wall to reach its goal.
    """
    environment.grid[0:4, 1] = 1

    pathfinder = AStar(environment)
    path = pathfinder.find_path()

    assert len(path) > 0
    assert len(path) - 1 == 8
    # Confirm that no position in the calculated path
    # overlaps with a static obstacle.
    for position in path:
        assert not environment.is_obstacle(position)

    # The drone must still reach the destination.
    assert path[-1] == environment.goal


def test_unreachable_goal(environment):
    """
    Test 3: A* should return an empty path when the goal
    is completely surrounded by obstacles.

    This verifies that the algorithm handles failure
    gracefully instead of entering an infinite loop.
    """
    environment.grid[3, 4] = 1
    environment.grid[4, 3] = 1

    pathfinder = AStar(environment)
    path = pathfinder.find_path()

    assert path == []


def test_same_start_and_goal(environment):
    """
    Test 4: If the drone is already at its destination,
    A* should return a path containing only that cell.
    """
    pathfinder = AStar(environment)

    path = pathfinder.find_path(
        start=(2, 2),
        goal=(2, 2)
    )

    assert path == [(2, 2)]


def test_valid_neighbor_movements(environment):
    """
    Test 5: Every movement in the route must be valid.

    Since diagonal movement is disabled, consecutive
    cells must have a Manhattan distance of exactly 1.
    """
    pathfinder = AStar(environment)
    path = pathfinder.find_path()

    for current, next_position in zip(path, path[1:]):

        row_difference = abs(
            current[0] - next_position[0]
        )

        col_difference = abs(
            current[1] - next_position[1]
        )

        # Exactly one coordinate must change by one cell.
        assert row_difference + col_difference == 1

        # The next position must be inside the grid
        # and must not contain a static obstacle.
        assert environment.is_walkable(next_position)
