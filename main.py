from serial_interface import SerialInterface
from occupancy_grid import OccupancyGrid
from frontier import select_frontier
from config import *

import time

serial = SerialInterface(SERIAL_PORT, BAUDRATE)
grid = OccupancyGrid(GRID_SIZE, GRID_RESOLUTION)

robot_pose = (0, 0, 0)

while True:
    packet = serial.read_packet()
    if packet is None:
        continue

    robot_pose = (packet["x"], packet["y"], packet["theta"])
    grid.update(
        robot_pose,
        (packet["dist"], packet["angle"])
    )

    target = select_frontier(grid.grid,
                             grid.world_to_grid(packet["x"], packet["y"]))

    if target:
        tx = (target[0] - grid.origin) * grid.res
        ty = (target[1] - grid.origin) * grid.res
        serial.send_waypoint(tx, ty)

    time.sleep(0.1)