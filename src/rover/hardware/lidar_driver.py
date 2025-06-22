from abc import ABC, abstractmethod
import launch.logging

class LidarDriver(ABC):

    def __init__(self, logger, topic:str, name="Lidar"):
        self.logger = logger
        self.topic = topic
        self.name = name
        self.logger.info(f"Initalized {self.name} on topic {self.topic}")

    def __str__(self):
        return f"[{self.name}]"

    @abstractmethod
    def start(self):
        raise NotImplementedError("start() muss überschrieben werden.")