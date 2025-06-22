from .sensor_interface import SensorInterface
import launch.logging

class LidarSensor(SensorInterface):
    def __init__(self,topic: str):
        self.lidar=None
        self.logger = launch.logging.get_logger('LidarSensor')
        self.logger.info(f"Lidar-Topic: {topic}")
        pass

    def read_data(self):
        pass

    def start(self):
        if self.lidar is not None:
            self.lidar.start()
        else:
            print(f"[{self.model}] not found");
        pass