import numpy as np
import pandas as pd
import plotly.graph_objects as go

DataBase = pd.read_csv("src/robot_data_optimized.csv")
SensorHeight = 0

X_global = []
Y_global = []
Z_global = []

robot_x = DataBase["x"].values
robot_y = DataBase["y"].values
robot_z = np.zeros(len(robot_x))

for index, row in DataBase.iterrows():

    x = row["x"]
    y = row["y"]
    theta = row["theta"]
    distance = row["distance"]
    pan = row["pan"]
    tilt = row["tilt"]

    total_yaw = theta + pan

    yaw_rad = np.radians(total_yaw)
    tilt_rad = np.radians(tilt)

    HorizontalProjection = distance * np.sin(tilt_rad)

    x_local =  HorizontalProjection * np.cos(yaw_rad)
    y_local =  HorizontalProjection * np.sin(yaw_rad)
    z_local = distance * np.cos(tilt_rad)

    x_global = x + x_local
    y_global = y + y_local
    z_global = z_local + SensorHeight

    X_global.append(x_global)
    Y_global.append(y_global)
    Z_global.append(z_global)

fig = go.Figure()

fig.add_trace(go.Scatter3d(
    x = X_global,
    y = Y_global,
    z = Z_global,
    mode='markers',
    marker=dict(
        size=2,
        color=Z_global,
        colorscale='Viridis',
        opacity=0.8
    ),
    name="Map Points"
))

fig.add_trace(go.Scatter3d(
    x=robot_x,
    y=robot_y,
    z=robot_z,
    mode='lines',
    line=dict(color='red', width=0),
    name="Robot's Path"
))

fig.update_layout(
    title="3D Environment Map",
    scene=dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z",
        aspectmode="manual",
        aspectratio=dict(x=1, y=1, z=0.5)
    ),
    width=900,
    height=800
)


fig.show()