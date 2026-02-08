import numpy as np
import pandas as pd
from scipy.ndimage import binary_dilation

DataBase = pd.read_csv("src/robot_data_optimized.csv")
X_global = DataBase['x'].values
Y_global = DataBase['y'].values

robot_pos = (X_global[-1], Y_global[-1])

def frontier_exploration(X_global, Y_global, robot_pos, grid_res=0.2):
    X_global = np.array(X_global)
    Y_global = np.array(Y_global)

    min_x, min_y = X_global.min(), Y_global.min()
    max_x, max_y = X_global.max(), Y_global.max()

    width = int(np.ceil((max_x - min_x) / grid_res)) + 5
    height = int(np.ceil((max_y - min_y) / grid_res)) + 5

    grid = np.zeros((height, width), dtype=np.uint8)

    ix = ((X_global - min_x) / grid_res).astype(int)
    iy = ((Y_global - min_y) / grid_res).astype(int)
    grid[iy, ix] = 1

    frontier = grid & binary_dilation(grid == 0)
    frontier_indices = np.argwhere(frontier)

    if len(frontier_indices) == 0:
        return None, None

    frontier_coords = np.array([
        (idx[1] * grid_res + min_x,
         idx[0] * grid_res + min_y)
        for idx in frontier_indices
    ])

    robot_x, robot_y = robot_pos
    dists = np.linalg.norm(
        frontier_coords - np.array([robot_x, robot_y]),
        axis=1
    )

    fx, fy = frontier_coords[np.argmin(dists)]

    return fx, fy


fx, fy = frontier_exploration(X_global, Y_global, robot_pos)
print(fx, fy)
