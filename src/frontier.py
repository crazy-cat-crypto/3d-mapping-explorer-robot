import numpy as np
from scipy.ndimage import label

def find_frontiers(grid):
    frontiers = np.zeros_like(grid)

    for x in range(1, grid.shape[0] - 1):
        for y in range(1, grid.shape[1] - 1):
            if grid[x, y] == 0:
                if -1 in grid[x-1:x+2, y-1:y+2]:
                    frontiers[x, y] = 1
    return frontiers


def select_frontier(grid, robot_pos):
    frontiers = find_frontiers(grid)
    labeled, num = label(frontiers)

    best_score = -1
    best_target = None

    for i in range(1, num + 1):
        cells = np.argwhere(labeled == i)
        cx, cy = cells.mean(axis=0)

        dx = cx - robot_pos[0]
        dy = cy - robot_pos[1]
        dist = np.hypot(dx, dy)

        info_gain = len(cells)
        score = info_gain / (dist + 1e-3)

        if score > best_score:
            best_score = score
            best_target = (cx, cy)

    return best_target
