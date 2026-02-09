
import os
import numpy as np
import pandas as pd
from scipy.ndimage import binary_dilation
os.chdir(os.path.dirname(os.path.abspath(__file__)))
DataBase = pd.read_csv("robot_data.csv")
X_global = DataBase['x'].values
Y_global = DataBase['y'].values

robot_pos = (X_global[-1], Y_global[-1])

def frontier_exploration():
    DataBase = pd.read_csv("robot_data.csv")
    X_global = DataBase['x'].values
    Y_global = DataBase['y'].values
    
    robot_pos = (X_global[-1], Y_global[-1])
    
    grid_res = 0.2
    
    X = np.array(X_global)
    Y = np.array(Y_global)
    
    min_x, min_y = X.min(), Y.min()
    max_x, max_y = X.max(), Y.max()
    
    width = int(np.ceil((max_x - min_x) / grid_res)) + 5
    height = int(np.ceil((max_y - min_y) / grid_res)) + 5
    
    grid = np.zeros((height, width), dtype=np.uint8)
    
    ix = ((X - min_x) / grid_res).astype(int)
    iy = ((Y - min_y) / grid_res).astype(int)
    grid[iy, ix] = 1
    
    frontier = grid & binary_dilation(grid == 0)
    frontier_indices = np.argwhere(frontier)
    
    if len(frontier_indices) == 0:
        return None, None
    
    frontier_coords = np.array([
        (idx[1] * grid_res + min_x, idx[0] * grid_res + min_y)
        for idx in frontier_indices
    ])
    
    rx, ry = robot_pos
    dists = np.linalg.norm(frontier_coords - np.array([rx, ry]), axis=1)
    
    fx, fy = frontier_coords[np.argmin(dists)]
    
    return fx, fy

x, y = frontier_exploration()
print(x, y)