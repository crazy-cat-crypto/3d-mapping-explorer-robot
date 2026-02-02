import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

DataBase = pd.read_csv(r"C:\Projects\3d-mapping-explorer-robot\src\robot_data.csv")
Poses = DataBase[['time', 'x', 'y']].values

def detect_loops(Poses, radius = 0.6, MinimumTimeGap = 25):
    loops = []

    for i in range(len(Poses)):

        t_i, x_i, y_i = Poses[i]

        for j in range(i):
            t_j, x_j, y_j = Poses[j]

            if abs(t_i - t_j) < MinimumTimeGap:
                continue


            if np.hypot(x_i - x_j, y_i - y_j) < radius:
                loops.append((j, i))
    return loops

loops = detect_loops(Poses)

edges = []

for i in range(1, len(Poses)):
    edges.append((i-1, i))


edges.extend(loops)

def GraphError(state, edges, Poses):

    state = state.reshape((-1, 2))
    error = []

    error.extend(state[0] - Poses[0, 1:3])

    for i,j in edges:

        if abs(i - j) > 1:
            dx, dy = 0, 0
        else:
            dx = Poses[j][1] - Poses[i][1]
            dy = Poses[j][2] - Poses[i][2]

        error.extend([
            (state[j][0] - state[i][0]) - dx,
            (state[j][1] - state[i][1]) - dy
        ])

    return np.array(error)

initial = Poses[:, 1:3].flatten()


result = least_squares(GraphError, initial, args=(edges, Poses))

optimized = result.x.reshape((-1,2))

