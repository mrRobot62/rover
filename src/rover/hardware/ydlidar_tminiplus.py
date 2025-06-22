# hardware/ydlidar_tminiplus.py
from .lidar_driver import LidarDriver


class YDLidarTFMini(LidarDriver):
    def __init__(self, logger, topic: str):
        super().__init__(logger, topic, "YDLidar.TFMiniPlus")

    def start(self):
        pass