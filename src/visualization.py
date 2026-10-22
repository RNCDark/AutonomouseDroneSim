
# src/visualization.py

import pygame

from src.config import (
    CELL_SIZE,
    GRID_WIDTH,
    GRID_HEIGHT,
    GRID_COLOR,
    BACKGROUND_COLOR,
    OBSTACLE_COLOR,
    DYNAMIC_OBSTACLE_COLOR,
    DRONE_COLOR,
    GOAL_COLOR,
    PATH_COLOR,
    WINDOW_TITLE,
)


class Visualization:
    """Draws the drone simulation and its live performance dashboard."""

    def __init__(self, environment):
        self.environment = environment

        pygame.init()

        # Reserve extra horizontal space for the dashboard.
        self.grid_width = environment.width * CELL_SIZE
        self.grid_height = environment.height * CELL_SIZE
        self.panel_width = 280

        self.screen = pygame.display.set_mode(
            (self.grid_width + self.panel_width, self.grid_height)
        )

        pygame.display.set_caption(WINDOW_TITLE)

        self.font = pygame.font.SysFont("consolas", 18)
        self.small_font = pygame.font.SysFont("consolas", 15)
        self.title_font = pygame.font.SysFont("consolas", 23, bold=True)

        # Store the time until which the rerouting message
        # should remain visible on the screen.
        self.reroute_message_until = 0

        # Remember the last observed reroute count so the
        # notification only triggers for a new rerouting event.
        self.previous_reroute_count = 0

    def cell_rect(self, position):
        """
        Convert a grid position into a Pygame rectangle.

        Our simulation uses (row, column), but Pygame
        draws objects using (x, y) pixel coordinates.
        """
        row, col = position

        return pygame.Rect(
            col * CELL_SIZE,
            row * CELL_SIZE,
            CELL_SIZE,
            CELL_SIZE
        )

    def draw_grid(self):
        """Draw the environment's cells and static obstacles."""

        for row in range(self.environment.height):
            for col in range(self.environment.width):
                position = (row, col)
                rect = self.cell_rect(position)

                if self.environment.is_obstacle(position):
                    pygame.draw.rect(
                        self.screen, OBSTACLE_COLOR, rect
                    )

                pygame.draw.rect(
                    self.screen, GRID_COLOR, rect, 1
                )

    def draw_path(self, drone):
        """Highlight the remaining A* route."""

        for position in drone.path[1:]:
            rect = self.cell_rect(position).inflate(-12, -12)

            pygame.draw.rect(
                self.screen,
                PATH_COLOR,
                rect,
                border_radius=5
            )

    def draw_dynamic_obstacles(self):
        """Display moving obstacles as red circles."""

        for position in self.environment.dynamic_obstacles:
            rect = self.cell_rect(position)

            pygame.draw.circle(
                self.screen,
                DYNAMIC_OBSTACLE_COLOR,
                rect.center,
                CELL_SIZE // 3
            )

    def draw_sensor_range(self, drone):
        """
        Highlight cells within the drone's sensor radius.

        Manhattan distance creates a diamond-shaped
        detection region on the grid.
        """

        row, col = drone.position
        detection_range = drone.sensor.detection_range

        overlay = pygame.Surface(
            (self.grid_width, self.grid_height),
            pygame.SRCALPHA
        )

        for dr in range(-detection_range, detection_range + 1):
            for dc in range(-detection_range, detection_range + 1):

                if abs(dr) + abs(dc) > detection_range:
                    continue

                target = (row + dr, col + dc)

                if not self.environment.is_within_bounds(target):
                    continue

                pygame.draw.rect(
                    overlay,
                    (50, 120, 220, 35),
                    self.cell_rect(target)
                )

        self.screen.blit(overlay, (0, 0))

    def draw_drone(self, drone):
        """Draw the drone at its current grid position."""

        rect = self.cell_rect(drone.position)

        # Main drone body.
        pygame.draw.circle(
            self.screen,
            DRONE_COLOR,
            rect.center,
            CELL_SIZE // 3
        )

        # White center makes the drone easy to identify.
        pygame.draw.circle(
            self.screen,
            (255, 255, 255),
            rect.center,
            4
        )

    def draw_goal(self):
        """Draw the destination as a green cell."""

        rect = self.cell_rect(self.environment.goal)

        pygame.draw.rect(
            self.screen,
            GOAL_COLOR,
            rect.inflate(-5, -5),
            border_radius=5
        )

    def draw_dashboard(self, drone, elapsed_time, paused=False):
        """Display live navigation metrics beside the grid."""

        panel_x = self.grid_width

        pygame.draw.rect(
            self.screen,
            (30, 35, 45),
            (
                panel_x,
                0,
                self.panel_width,
                self.grid_height
            )
        )

        title = self.title_font.render(
            "DRONE NAVIGATION",
            True,
            (255, 255, 255)
        )

        self.screen.blit(title, (panel_x + 15, 25))

        information = [
            ("State", drone.state.value),
            ("Position", str(drone.position)),
            ("Destination", str(drone.goal)),
            ("Steps", str(drone.steps_taken)),
            ("Reroutes", str(drone.reroute_count)),
            ("Pauses", str(drone.pause_count)),
            ("Elapsed Time", f"{elapsed_time:.2f}s"),
            ("Route Remaining", str(max(0, len(drone.path) - 1))),
            ("Nodes Explored", str(drone.pathfinder.nodes_explored)),
            ("Dynamic Obstacles",
             str(len(self.environment.dynamic_obstacles))),
        ]

        y = 80

        for label, value in information:
            label_surface = self.small_font.render(
                label, True, (170, 180, 195)
            )

            value_surface = self.font.render(
                value, True, (255, 255, 255)
            )

            self.screen.blit(label_surface, (panel_x + 15, y))
            self.screen.blit(value_surface, (panel_x + 15, y + 20))

            y += 43

        instructions = [
            "SPACE - Pause / Resume",
            "S - Save Screenshot",
            "ESC - Exit"
        ]

        y = self.grid_height - 75

        for instruction in instructions:
            surface = self.small_font.render(
                instruction, True, (180, 190, 205)
            )
            self.screen.blit(surface, (panel_x + 15, y))
            y += 22

        if paused:
            label = self.font.render(
                "SIMULATION PAUSED",
                True,
                (255, 190, 50)
            )
            self.screen.blit(label, (panel_x + 15, 60))

    def render(self, drone, elapsed_time, paused=False):
        """Draw one complete frame of the simulation."""

        self.screen.fill(BACKGROUND_COLOR)

        self.draw_grid()
        self.draw_sensor_range(drone)
        self.draw_path(drone)
        self.draw_dynamic_obstacles()
        self.draw_goal()
        self.draw_drone(drone)
        # Display recent autonomous rerouting decisions
        self.draw_reroute_notification(drone)
        self.draw_dashboard(drone, elapsed_time, paused)

        # Display the completed frame.
        pygame.display.flip()

    def close(self):
        """Release Pygame resources."""
        pygame.quit()
    
    def draw_reroute_notification(self, drone):
        """
        Display a temporary notification when A* replanning occurs.

        The message remains visible for 1.5 seconds so viewers
        can recognize the drone's autonomous decision.
        """

        # Pygame tracks elapsed application time in milliseconds.
        current_time = pygame.time.get_ticks()

        # Detect whether the drone performed another reroute.
        if drone.reroute_count > self.previous_reroute_count:

            # Keep the notification visible for 1500 milliseconds.
            self.reroute_message_until = current_time + 1500

            # Update the stored counter.
            self.previous_reroute_count = drone.reroute_count

        # Display the notification while its timer is active.
        if current_time < self.reroute_message_until:

            message = self.font.render(
                "REROUTING - OBSTACLE DETECTED",
                True,
                (255, 255, 255)
            )

            # Create a dark background for readability.
            background = pygame.Rect(
                70,
                15,
                500,
                42
            )

            pygame.draw.rect(
                self.screen,
                (180, 75, 35),
                background,
                border_radius=8
            )

            # Center the message inside the notification.
            self.screen.blit(
                message,
                message.get_rect(center=background.center)
            )
