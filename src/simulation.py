
# src/simulation.py

import os
import pygame

from src.config import (
    FPS,
    DRONE_SPEED,
    DYNAMIC_OBSTACLE_INTERVAL,
)
from src.environment import Environment
from src.drone import Drone, DroneState
from src.visualization import Visualization
from src.metrics import SimulationMetrics


class Simulation:
    """Coordinates drone navigation, obstacle movement, and rendering."""

    def __init__(self):
        self.environment = Environment()
        self.drone = Drone(self.environment)

        # Create the metrics collector before navigation begins.
        self.metrics = SimulationMetrics(
            self.environment,
            self.drone
        )

        # Prevent duplicate result exports after arrival.
        self.results_saved = False
        # Calculate the initial route before generating
        # dynamic obstacles, matching our previous test.
        self.drone.start()

        self.environment.initialize_dynamic_obstacles(
            excluded_positions=set(self.drone.path)
        )

        self.visualization = Visualization(self.environment)

        self.clock = pygame.time.Clock()

        # Time accumulators allow movement to occur at
        # controlled intervals instead of every rendered frame.
        self.drone_timer = 0.0
        self.obstacle_timer = 0.0
        self.elapsed_time = 0.0

        self.paused = False
        self.running = True

        # At four cells per second, the drone attempts
        # one movement every 0.25 seconds.
        self.drone_interval = 1.0 / DRONE_SPEED

    def handle_events(self):
        """Handle window closing and keyboard controls."""

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    self.running = False

                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused

                elif event.key == pygame.K_s:
                    os.makedirs(
                        "evidence/screenshots",
                        exist_ok=True
                    )

                    pygame.image.save(
                        self.visualization.screen,
                        "evidence/screenshots/simulation.png"
                    )

                    print("Screenshot saved.")

    def update(self, dt):
        """
        Advance the simulation based on elapsed time.

        dt represents the time passed since the last frame.
        This keeps simulation speed independent of rendering FPS.
        """

        if self.paused:
            return

        if self.drone.state == DroneState.ARRIVED:
            return

        self.elapsed_time += dt
        self.drone_timer += dt
        self.obstacle_timer += dt

        # Move obstacles at their configured interval.
        while self.obstacle_timer >= DYNAMIC_OBSTACLE_INTERVAL:

            self.environment.move_dynamic_obstacles(
                drone_position=self.drone.position
            )

            self.obstacle_timer -= DYNAMIC_OBSTACLE_INTERVAL

        # Update the drone at its configured movement rate.
        while self.drone_timer >= self.drone_interval:

            self.drone.update(
                dynamic_obstacles=self.environment.dynamic_obstacles
            )

            self.drone_timer -= self.drone_interval

            if self.drone.state == DroneState.ARRIVED:
                 # Export results only once when the destination
                # is reached, even if the simulation continues
                # rendering additional frames.
                if not self.results_saved:

                    results = self.metrics.calculate(
                        elapsed_time=self.elapsed_time,
                        scenario="dynamic"
                    )

                    print("\nSimulation Results")
                    print("-" * 40)

                    for metric, value in results.items():
                        print(f"{metric}: {value}")

                    # Save the experiment data for later analysis.
                    self.metrics.save_to_csv(
                        results,
                        "results/simulation_metrics.csv"
                    )

                    self.results_saved = True

                break


    def run(self):
        """Start the main Pygame simulation loop."""

        print("Starting Autonomous Drone Simulation...")
        print("Press SPACE to pause or resume.")
        print("Press S to save a screenshot.")
        print("Press ESC to exit.")

        try:
            while self.running:

                # Limit rendering to the configured FPS.
                dt = self.clock.tick(FPS) / 1000.0

                self.handle_events()
                self.update(dt)

                self.visualization.render(
                    self.drone,
                    self.elapsed_time,
                    self.paused
                )

        finally:
            self.visualization.close()
