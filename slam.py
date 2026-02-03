from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

class SLAMSystem:
    def __init__(self):
        self.laser = Laser(180, 4.0)
        self.slam = RMHC_SLAM(self.laser, 400, 20.0)
        self.pose = (0, 0, 0)

    def update(self, scan, odometry):
        self.slam.update(scan, odometry)
        self.pose = self.slam.getpos()
        return self.pose
