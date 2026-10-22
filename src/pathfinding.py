
# src/pathfinding.py

import heapq
from itertools import count


class AStar:
    """A* pathfinding for a four-directional grid."""

    def __init__(self, environment):
        self.environment = environment
        self.nodes_explored = 0

    @staticmethod
    def heuristic(position, goal):
        """Calculate Manhattan distance."""

        return (
            abs(position[0] - goal[0])
            + abs(position[1] - goal[1])
        )

    @staticmethod
    def reconstruct_path(came_from, current):
        """Reconstruct the path from goal to start."""

        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def find_path(self, start=None, goal=None, blocked_cells=None):
        """Return the shortest path or an empty list if unreachable."""

        if start is None:
            start = self.environment.start

        if goal is None:
            goal = self.environment.goal

        self.nodes_explored = 0

        # Dynamic obstacles are temporary blocked positions.
        # We use a set for efficient membership checks.
        blocked_cells = set() if blocked_cells is None else set(blocked_cells)

        # The starting and goal positions must be traversable.
        # The drone cannot plan through static or known dynamic obstacles.
        if (
            not self.environment.is_walkable(start)
            or not self.environment.is_walkable(goal)
            or start in blocked_cells
            or goal in blocked_cells
        ):
            return []

        if start == goal:
            return [start]

        # Priority queue: (f_score, tie_breaker, position)
        counter = count()

        open_set = []
        heapq.heappush(
            open_set,
            (self.heuristic(start, goal), next(counter), start)
        )

        came_from = {}
        g_score = {start: 0}

        closed_set = set()

        # Continue searching while there are candidate cells to explore.
        while open_set: 
            # Remove the cell with the lowest estimated total path cost.
            # The two underscores discard the priority and tie-breaker,
            # since we only need the cell's position here.
            _, _, current = heapq.heappop(open_set)

            # Skip cells that have already been fully processed.
            # The same cell may appear in the queue multiple times
            # when a cheaper route to it has been discovered.
            if current in closed_set:
                continue
            # Count how many unique cells A* actually expands.
            # We'll use this metric when evaluating search efficiency.
            self.nodes_explored += 1
            # If the destination has been reached, reconstruct
            # the complete route using the stored predecessor links.
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # Mark this cell as processed so it isn't expanded again.
            closed_set.add(current)

            # Examine each accessible neighboring cell.
            for neighbor in self.environment.get_neighbors(current):
                # Don't revisit cells that have already been processed.
                if neighbor in closed_set:
                    continue

                # Ignore cells occupied by detected dynamic obstacles.
                if neighbor in blocked_cells:
                    continue

                # Every movement costs one unit in our grid.
                # Calculate the cost of reaching this neighbor
                # through the current cell.
                tentative_g = g_score[current] + 1

                # Only update the neighbor when the new route is cheaper
                # than the best route previously discovered.
                if tentative_g < g_score.get(neighbor, float("inf")):

                    # Remember which cell leads to this neighbor.
                    # These links will reconstruct the final route.
                    came_from[neighbor] = current

                    # Store the cheapest known cost from the start.
                    g_score[neighbor] = tentative_g

                    # A* combines the actual travel cost (g)
                    # with the estimated remaining distance (h).
                    f_score = tentative_g + self.heuristic(
                        neighbor, goal
                    )

                    # Add the neighbor to the priority queue.
                    # Cells with lower estimated costs are considered first.
                    heapq.heappush(
                        open_set,
                        (f_score, next(counter), neighbor)
                    )

        return []
