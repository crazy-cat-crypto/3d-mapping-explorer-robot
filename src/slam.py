import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial import KDTree

ErrorWeight = 10.0
DataBase = pd.read_csv("src/robot_data.csv")
Poses = DataBase[['time', 'x', 'y', 'theta']].values

Poses[:, 3] = np.radians(Poses[:, 3])

def detect_loops(Poses, radius = 0.6, MinimumTimeGap = 25):
    xy = Poses[:, 1:3]
    time = Poses[:, 0]

    tree = KDTree(xy)
    loops = []

    for i, (t, x, y, theta) in enumerate(Poses):
        neighbors = tree.query_ball_point([x,y], r=radius)

        for j in neighbors:
            if j < i and abs(time[i] - time[j] > MinimumTimeGap):
                loops.append((j, i))

    return loops

loops = detect_loops(Poses)
print(f"Loops : {loops}")

edges = []

for i in range(1, len(Poses)):
    edges.append((i-1, i))


edges.extend(loops)

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def GraphError(state, edges, Poses):

    state = state.reshape((-1, 3))
    errors = []

    for i, j in edges:
        xi, yi, thi = state[i]
        xj, yj, thj = state[j]

        # Measured
        mi_x, mi_y, mi_th = Poses[i, 1], Poses[i, 2], Poses[i, 3]
        mj_x, mj_y, mj_th = Poses[j, 1], Poses[j, 2], Poses[j, 3]

        # Measured (relative)
        cos_mi = np.cos(mi_th)
        sin_mi = np.sin(mi_th)
        dx_m = cos_mi * (mj_x - mi_x) + sin_mi * (mj_y - mi_y)
        dy_m = -sin_mi * (mj_x - mj_x) + cos_mi * (mj_y - mi_y)
        dth_m = wrap_angle(mj_th - mi_th)

        # Predicted
        cos_pi = np.cos(thi)
        sin_pi = np.sin(thi)
        dx_p = cos_pi * (xj - xi) + sin_pi * (yj - yi)
        dy_p = -sin_pi * (xj - xi) + cos_pi *(yj - yi)
        dth_p = wrap_angle(thj - thi)


        errors.extend([
            dx_p - dx_m,
            dy_p - dy_m,
            wrap_angle(dth_p - dth_m) * ErrorWeight
        ])

    DriftError = (state[0] - Poses[0, 1:4]) * ErrorWeight
    errors.extend(DriftError)

    return np.array(errors)

InitialState = Poses[:, 1:4].flatten()


result = least_squares(
    GraphError, 
    InitialState, 
    args=(edges, Poses),
    verbose=2
    )

optimized = result.x.reshape((-1,3))

CopyData = DataBase.copy()

CopyData['x'] = optimized[:, 0]
CopyData['y'] = optimized[:, 1]
CopyData['theta'] = np.degrees(optimized[:, 2])


CopyData.to_csv(
    "src/robot_data_optimized.csv",
    index=False
)

plt.figure(figsize=(8, 6))

plt.plot(Poses[:,1], Poses[:,2], 'r--', label='Original')

for i, j in loops:
    plt.plot(
        [Poses[i][1], Poses[j][1]],
        [Poses[i][2], Poses[j][2]],
        'b:',
        alpha=0.5
    )

plt.plot(
    optimized[:,0],
    optimized[:,1],
    'g',
    linewidth=2,
    label='Optimized'
)

plt.quiver(
    optimized[:,0],
    optimized[:,1],
    np.cos(optimized[:,2]),
    np.sin(optimized[:,2]),
    scale=20,
    width=0.003,
    color='black'
)

plt.legend()
plt.axis('equal')
plt.grid(True)
plt.title("SLAM with Loop Closure")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()