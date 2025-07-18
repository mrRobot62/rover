from rover_interfaces.srv import I2CESP32Communication
from .ESP32CommandsV1 import CommandID, SubCommandID, ESP32PINS, SERVICE_RESPONSE
import rclpy
from typing import List
from rclpy.node import Node

"""
driver_controller_node
 └── ESP32Driver
      └── ruft ROS-Service /i2c/write und /i2c/read auf
            ↳ handled vom i2c_node → dieser macht echten I2C-Zugriff
"""


class ESP32Client:
    """
    Der ESP-Driver ist das Zwischenglied zwischen dem driver_controller_node und
    dem I2C_Node
    """
    def __init__(self, node: Node, channel):
        self.node = node
        self.client = node.create_client(I2CESP32Communication, channel)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(f'{channel} not available...')

    def write_servo_sv(self, steering: float, velocity: float) -> int: 
        """
        Generiert einen Befehlssatz für Steering und Velocity
        CommandID.SERVO_WRITE & SubCommandID.SCMD_SERVO_SPEED_POSITION
        """
        request = I2CESP32Communication.Request()
        request.device = "ESP32"
        request.command = CommandID.SERVO_WRITE.value
        request.subcommand = SubCommandID.SCMD_SERVO_SPEED_POSITION.value
        request.fvalues = []
        request.fvalues[0] = steering
        request.fvalues[1] = velocity
        request.ivalues = []

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.get_logger().info(f"write_servo_sv: steering={result.fvalues[0]}, velocity={result.fvalues[1]}")
        result = future.result()

        if result is None:
            self.node.get_logger().error(f"ROS2-Service nicht verfügbar")
            return SERVICE_RESPONSE.ROS_SERVICE_NOT_AVAILABEL.value
        
        if not result.ivalues:
            self.node.get_logger().warn(f"ESP32 sendet keinen Response - in diesem Fall kein Problem")
            return SERVICE_RESPONSE.SLAVE_RESPONSE_EMPTY.value
        return result.ivalues[0]  # z. B. 0 = OK, <>0 = ESP32-Fehler

    def write_servo(self, cmd, scmd, fvalues: List[float], ivalues: List[int]) -> int:
        """
        Generiert einen Befehlssatz basierend auf cmd udn scmd. Die dazugehörigen Werte sehen in
        fvalues und ivalues
        """

        # return  result.ivalues[0]  # z. B. 0 = OK, <>0 = ESP32-Fehler
        return 0

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