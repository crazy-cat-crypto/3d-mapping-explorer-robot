import numpy as np
import math

class OccupancyGrid:
    def __init__(self, size, resolution):
        self.size = size
        self.res = resolution
        self.grid = -np.ones((size, size)) 
        self.origin = size // 2

    def world_to_grid(self, x, y):
        gx = int(self.origin + x / self.res)
        gy = int(self.origin + y / self.res)
        return gx, gy

    def update(self, pose, measurement):
        x, y, theta = pose
        d, a = measurement

        angle = math.radians(theta + a)
        ox = x + d * math.cos(angle)
        oy = y + d * math.sin(angle)

        gx, gy = self.world_to_grid(ox, oy)
        rx, ry = self.world_to_grid(x, y)

        if 0 <= gx < self.size and 0 <= gy < self.size:
            self.grid[gx, gy] = 1

        self.raytrace(rx, ry, gx, gy)

    def raytrace(self, x0, y0, x1, y1):
        points = zip(
            np.linspace(x0, x1, 20).astype(int),
            np.linspace(y0, y1, 20).astype(int)
        )
        for x, y in points:
            if 0 <= x < self.size and 0 <= y < self.size:
                if self.grid[x, y] == -1:
                    self.grid[x, y] = 0
