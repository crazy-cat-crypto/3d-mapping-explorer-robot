import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation

# -------------------------------
# Load your robot map
# -------------------------------
DataBase = pd.read_csv("src/robot_data.csv")
X_global = DataBase['x'].values
Y_global = DataBase['y'].values

# Robot current position (last point in the path)
robot_pos = (X_global[-1], Y_global[-1])

# -------------------------------
# Frontier exploration function
# -------------------------------
def frontier_exploration(X_global, Y_global, robot_pos, grid_res=0.2):
    """
    Simple 2D frontier exploration.

    Returns:
    - closest frontier point
    - all frontier points for visualization
    """
    X_global = np.array(X_global)
    Y_global = np.array(Y_global)

    # Grid limits
    min_x, min_y = X_global.min(), Y_global.min()
    max_x, max_y = X_global.max(), Y_global.max()
    width = int(np.ceil((max_x - min_x) / grid_res)) + 5
    height = int(np.ceil((max_y - min_y) / grid_res)) + 5

    # Occupancy grid
    grid = np.zeros((height, width), dtype=np.uint8)
    ix = ((X_global - min_x) / grid_res).astype(int)
    iy = ((Y_global - min_y) / grid_res).astype(int)
    grid[iy, ix] = 1

    # Find frontiers: explored cells next to unknown
    frontier = grid & binary_dilation(grid == 0)
    frontier_indices = np.argwhere(frontier)
    if len(frontier_indices) == 0:
        return None, np.array([])

    frontier_coords = np.array([
        (idx[1]*grid_res + min_x, idx[0]*grid_res + min_y)
        for idx in frontier_indices
    ])

    # Closest frontier to robot
    robot_x, robot_y = robot_pos
    dists = np.linalg.norm(frontier_coords - np.array([robot_x, robot_y]), axis=1)
    closest_idx = np.argmin(dists)

    return tuple(frontier_coords[closest_idx]), frontier_coords

# -------------------------------
# Run frontier exploration
# -------------------------------
next_frontier, all_frontiers = frontier_exploration(X_global, Y_global, robot_pos)
print("Next frontier to explore:", next_frontier)

# -------------------------------
# Visualization
# -------------------------------
plt.figure(figsize=(8, 8))
plt.scatter(X_global, Y_global, c='blue', s=5, label='Explored Points')
if all_frontiers.size > 0:
    plt.scatter(all_frontiers[:,0], all_frontiers[:,1], c='orange', s=10, label='Frontiers')
if next_frontier is not None:
    plt.scatter(next_frontier[0], next_frontier[1], c='red', s=50, marker='*', label='Next Frontier')
plt.scatter(robot_pos[0], robot_pos[1], c='green', s=50, marker='o', label='Robot')
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("2D Frontier Exploration")
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
