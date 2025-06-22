# hardware/ydlidar_tminiplus.py
from .lidar_driver import LidarDriver

class XV11Lidar(LidarDriver):
    def __init__(self, logger, topic: str):
        super().__init__(logger, topic, "XV11-Lidar")
        self.logger = logger

    def start(self):
        pass