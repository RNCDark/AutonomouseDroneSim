
# src/config.py

# Environment configuration
GRID_WIDTH = 20
GRID_HEIGHT = 20
CELL_SIZE = 32

# Starting and destination coordinates (row, column)
START_POSITION = (0, 0)
GOAL_POSITION = (19, 19)

# Obstacle configuration
OBSTACLE_DENSITY = 0.20
RANDOM_SEED = 42

# Dynamic obstacle configuration
NUM_DYNAMIC_OBSTACLES = 5
DYNAMIC_OBSTACLE_INTERVAL = 0.5

# Drone configuration
DRONE_SPEED = 4.0  # Grid cells per second
SENSOR_RANGE = 3   # Detection range in grid cells

# Visualization configuration
FPS = 60
WINDOW_TITLE = "Autonomous Drone Navigation Simulation"

# Colors (RGB)
BACKGROUND_COLOR = (245, 245, 245)
GRID_COLOR = (210, 210, 210)
OBSTACLE_COLOR = (70, 70, 70)
DYNAMIC_OBSTACLE_COLOR = (220, 70, 70)
DRONE_COLOR = (50, 120, 220)
GOAL_COLOR = (50, 180, 100)
PATH_COLOR = (255, 190, 50)
