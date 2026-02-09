from flask import Flask, request, jsonify
import numpy as np
from scipy.ndimage import label

app = Flask(__name__)

grid = None
robot_pos = None

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
        if len(cells) < 4:
            continue9
        dists = np.linalg.norm(cells - robot_pos, axis=1)
        idx = np.argmin(dists)
        target = tuple(cells[idx])
        info_gain = len(cells)
        dist = dists[idx]
        score = info_gain / (dist + 1e-3)
        if score > best_score:
            best_score = score
            best_target = target
    return best_target, frontiers

@app.route("/update", methods=["POST"])
def update_grid():
    global grid, robot_pos
    data = request.json
    robot_pos = tuple(data["robot_pos"])
    grid = np.array(data["grid"])
    return jsonify({"status": "ok"})

@app.route("/get_target", methods=["GET"])
def get_target():
    if grid is None or robot_pos is None:
        return jsonify({"target": None})
    target, frontiers = select_frontier(grid, robot_pos)
    return jsonify({"target": target, "frontiers": frontiers.tolist()})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
