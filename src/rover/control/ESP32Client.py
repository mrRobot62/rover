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


    # def write_servo(self, steering: float, velocity: float, retries:int = 3) -> int: 
    #     """
    #     Generiert einen Befehlssatz für Steering und Velocity
    #     CommandID.SERVO_WRITE & SubCommandID.SCMD_SERVO_SPEED_POSITION
    #     """
    #     for attempt in range(retries):
    #         self.node.get_logger().info(f"[write_servo] Versuch {attempt+1}/{retries}")
    #         request = I2CESP32Communication.Request()
    #         request.device = "ESP32"
    #         request.command = CommandID.SERVO_WRITE.value
    #         request.subcommand = SubCommandID.SCMD_SERVO_SPEED_POSITION.value
    #         request.fvalues = [steering, velocity]
    #         request.ivalues = []

    #         future = self.client.call_async(request)
    #         if not rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0):
    #             self.node.get_logger().warn(f"[write_servo] Timeout bei Versuch {attempt+1}")
    #             time.sleep(1.0)
    #             continue

    #         result = future.result()
    #         if result is None:
    #             self.node.get_logger().error("[write_servo] Kein Ergebnis vom Service")
    #             return SERVICE_RESPONSE.ROS_SERVICE_NOT_AVAILABEL.value

    #         if result.ivalues and result.ivalues[0] == SERVICE_RESPONSE.NOT_READY:
    #             self.node.get_logger().warn("[write_servo] Service noch nicht aktiviert – warte und retry")
    #             time.sleep(1.0)
    #             continue

    #         if not result.ivalues:
    #             self.node.get_logger().warn("[write_servo] Kein Response vom ESP32 – ignoriere")
    #             return SERVICE_RESPONSE.SLAVE_RESPONSE_EMPTY.value

    #         self.node.get_logger().info("[write_servo] Erfolgreich abgeschlossen")
    #         return result.ivalues[0]

    #     self.node.get_logger().error("[write_servo] Alle Versuche fehlgeschlagen")
    #     return SERVICE_RESPONSE.ROS_SERVICE_NOT_AVAILABEL.value
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


        # Serviceaufruf
        # future = self.client.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future)

        # self.node.get_logger().info(f"➡️ [REQUEST] : '{request}'")
        # if future.result() is not None:
        #     self.get_logger().info(f'⬅️ [RESPONSE] Received: {future.result()}')
        #     return future.result().ivalues[0]
        # else:
        #     self.get_logger().error('❗ [RESPONSE] Service call failed.')

        future = self.client.call_async(request)

        def callback_future(fut):
            if fut.result() is not None:
                self.node.get_logger().info(f'⬅️ [RESPONSE] Received: {fut.result()}')
            else:
                self.node.get_logger().error('❗ [RESPONSE] Service call failed.')

        future.add_done_callback(callback_future)


        return SERVICE_RESPONSE.SLAVE_RESPONSE_EMPTY
    
            # # Serviceaufruf
            # future = self.client.call_async(request)

            # # Warten auf Antwort ohne Blockade
            # timeout = 2.0  # Sekunden
            # start = time.time()
            # while rclpy.ok() and not future.done():
            #     rclpy.spin_once(self.node, timeout_sec=0.1)
            #     if time.time() - start > timeout:
            #         self.node.get_logger().warn(f"[write_servo] Timeout bei Versuch {attempt}")
            #         break

            # # Ergebnis prüfen
            # if not future.done():
            #     time.sleep(1.0)
            #     continue

            # result = future.result()
            # if result is None:
            #     self.node.get_logger().error("[write_servo] Kein Ergebnis vom Service (None)")
            #     return SERVICE_RESPONSE.ROS_SERVICE_NOT_AVAILABEL.value

            # # Prüfung auf "nicht aktiv"
            # if result.ivalues and result.ivalues[0] == SERVICE_RESPONSE.NOT_READY:
            #     self.node.get_logger().warn("[write_servo] Service noch nicht aktiviert – retry")
            #     time.sleep(1.0)
            #     continue

            # Prüfung auf leeren Response
            # if not result.ivalues:
            #     self.node.get_logger().warn("[write_servo] Leerer Response vom ESP32")
            #     return SERVICE_RESPONSE.SLAVE_RESPONSE_EMPTY.value

            # # Erfolgreicher Abschluss
            # self.node.get_logger().info("[write_servo] Erfolgreich abgeschlossen")
            # self.node.get_logger().debug(f"[write_servo] Response: {result.ivalues}")

        return SERVICE_RESPONSE.ROS_SERVICE_NOT_AVAILABEL.value

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