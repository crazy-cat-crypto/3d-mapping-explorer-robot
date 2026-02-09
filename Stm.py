import numpy as np
import pandas as pd
from scipy.ndimage import label

def find_frontiers(grid):
    frontiers = np.zeros_like(grid)

    for x in range(1, grid.shape[0] - 1):
        for y in range(1, grid.shape[1] - 1):
            if grid[x, y] == 0:
                if -1 in grid[x-1:x+2, y-1:y+2]:
                    frontiers[x, y] = 1

    return frontiers


def frontier_exploration():
    grid = pd.read_csv("robot_data.csv", header=None).values

    robot_x = int(grid.shape[0] / 2)
    robot_y = int(grid.shape[1] / 2)
    robot_pos = np.array([robot_x, robot_y])

    frontiers = find_frontiers(grid)
    labeled, num = label(frontiers)

    best_score = -1
    best_target = None

    for i in range(1, num + 1):
        cells = np.argwhere(labeled == i)

        if len(cells) < 4:
            continue

        dists = np.linalg.norm(cells - robot_pos, axis=1)
        idx = np.argmin(dists)
        target = cells[idx]

        info_gain = len(cells)
        dist = dists[idx]
        score = info_gain / (dist + 1e-3)

        if score > best_score:
            best_score = score
            best_target = target

    if best_target is None:
        return None, None

    x, y = int(best_target[0]), int(best_target[1])
    return x, y
