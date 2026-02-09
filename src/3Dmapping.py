import numpy as np
import pandas as pd
import plotly.graph_objects as go

DataBase = pd.read_csv("src/robot_data_optimized.csv")
SensorHeight = 10.5

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

fig.add_trace(go.Mesh3d(
    x=X_global,
    y=Y_global,
    z=Z_global,
    opacity=0.3,      
    color='cyan',     
    alphahull=7,       
    name="Reconstructed Object"
))

fig.add_trace(go.Scatter3d(
    x=X_global,
    y=Y_global,
    z=Z_global,
    mode='markers',
    marker=dict(
        size=1, 
        color=Z_global,
        colorscale='Viridis',
        opacity=1 
    ),
    name="Raw Sensor Hits"
))

fig.add_trace(go.Scatter3d(
    x=DataBase["x"],
    y=DataBase["y"],
    z=np.zeros(len(DataBase)),
    mode='lines',
    line=dict(color='red', width=4),
    name="Robot Path"
))

fig.update_layout(
    scene=dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z",
        aspectmode="data"
    ),
    title="3D Mapping: Mesh + Point Cloud"
)

fig.show()