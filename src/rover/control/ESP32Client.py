from rover_interfaces.srv import I2CESP32Communication
from .ESP32CommandsV1 import CommandID, SubCommandID, ESP32PINS, SERVICE_RESPONSE
import rclpy
from typing import List
from rclpy.node import Node
from ..rover_exceptions import *
import time

"""
driver_controller_node
 └── ESP32DriverClient
      └── ruft ROS-Service : I2CESP32Communication auf
            ↳ dieser Service wird vom I2CNode überwacht
"""


class ESP32Client:
    """
    Der ESP-Driver ist das Zwischenglied zwischen dem driver_controller_node und
    dem I2C_Node
    """
    def __init__(self, node: Node, channel, timeout:float=5.0):
        self.node = node
        self.class_name =  self.__class__.__name__


        max_wait_time = timeout  # Sekunden
        start_time = time.time()
        self.__is_available=False
        self.node.get_logger().info(f"[{self.class_name}] waiting for I2CESP32Communication")
        # einen Serive-Client erstellen
        # nur nodes können das, daher auch der zugriff über das Node
        self.client = self.node.create_client(I2CESP32Communication, channel)
        while not self.client.wait_for_service(timeout_sec=1.0):
            elapsed = time.time() - start_time
            self.node.get_logger().warn(f"[{self.class_name}] warte auf Service '{channel}' ({elapsed:.1f}s)")
            if elapsed > max_wait_time:
                raise ServiceNotAvailableException(f"[{self.class_name}] I2CESP32Communication channel: {channel} not available nach {elapsed:.1f}s")

        self.node.get_logger().info(f"[{self.class_name}] I2CESP32Communication available mit channel: {channel}")

    def write_servo(self, steering: float, velocity: float, retries: int = 3) -> int:
        """
        Sendet Steering- und Velocity-Werte an den ESP32 über den I2C-Service.
        Nutzt Retry-Logik und Timeout, um blockierende Aufrufe zu vermeiden.
        """
        # Request vorbereiten
        request = I2CESP32Communication.Request()
        request.device = "ESP32"
        request.command = CommandID.SERVO_WRITE.value
        request.subcommand = SubCommandID.SCMD_SERVO_SPEED_POSITION.value
        request.fvalues = [steering, velocity]
        request.ivalues = []

        future = self.client.call_async(request)
        #
        # sieht merkwürdig aus, ist aber notwendig, das ros2 kein async nutzt sondern anders arbeitet
        # um ein response sauber empfangen werden kann muß es entkoppelt werden und über
        # diesen callback funktioiert das einwandfrei
        #
        def callback_future(fut):
            if fut.result() is not None:
                self.node.get_logger().info(f'⬅️ [RESPONSE] Received: {fut.result()}')
            else:
                self.node.get_logger().error('❗ [RESPONSE] Service call failed.')

        future.add_done_callback(callback_future)


        return SERVICE_RESPONSE.SLAVE_RESPONSE_EMPTY

    def shutdown(self):
        if self.client is not None:
            self.node.destroy_service(self.client)
        

    def read_servo(self):
        request = I2CESP32Communication.Request()
        request.device = "ESP32"
        request.command = 1
        request.subcommand = 2

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        if result:
            return result.fvalues
        else:
            return [0.0, 0.0]