
# rover/src/rover/sensor_node.py

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher

from std_msgs.msg import Float32

from .sensors.lidar_sensor import LidarSensor
from .sensors.battery_sensor import BatterySensor
from rover_interfaces.msg import Battery

import random

class SensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_node')
        self.get_logger().info('SensorNode constructed')

        self.lidar = None
        self.batterySensor = None
        self.timer = None
        self.battery_publisher = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_configure()')

        # Parameter auslesen
        lidar_topic = self.declare_parameter("lidar_topic", "/scan").get_parameter_value().string_value

        # Initialisierung Lidar (z. B. für Softkopplung)
        self.lidar = LidarSensor(lidar_topic)

        # BatterySensor initialisieren
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('battery_critical', '13.1')
        self.declare_parameter('battery_low', '13.4')
        self.declare_parameter('battery_full', '16.2')
        self.battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        self.battery_critical = self.get_parameter('battery_critical').get_parameter_value().double_value
        self.battery_low = self.get_parameter('battery_low').get_parameter_value().double_value
        self.battery_full = self.get_parameter('battery_full').get_parameter_value().double_value
        try:
            self.batterySensor = BatterySensor(self.get_logger(), chan=0, gain=1)
        except Exception as err:
            self.get_logger().error(f"BatterySensor konnte nicht initialisiert werden: {err}")
            self.batterySensor = None

        # Publisher vorbereiten (aber noch nicht aktiviert)
        # Bemerkung: es wird explizit hier keine publish der Lidardaten durchgeführt, da der Lidar selber ein Topic-Publisher ist.
        #
        self.battery_publisher = self.create_lifecycle_publisher(Battery, self.battery_topic, 10)
        

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_activate()')

        # Publisher aktivieren
        self.battery_publisher.on_activate(state)

        # Timer starten
        self.timer = self.create_timer(10.0, self.read_and_publish_voltage)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_deactivate()')

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        self.battery_publisher.on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_cleanup()')

        # Publisher zerstören
        if self.battery_publisher is not None:
            self.destroy_publisher(self.battery_publisher)
            self.battery_publisher = None

        # BatterySensor aufräumen
        if self.batterySensor is not None:
            try:
                self.batterySensor.ads.i2c_device.i2c.unlock()
            except Exception as e:
                self.get_logger().warn(f'Konnte I2C nicht korrekt freigeben: {e}')
            self.batterySensor = None

        self.lidar = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_shutdown()')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('[SensorNode] Fehlerzustand!')
        return TransitionCallbackReturn.SUCCESS

    def read_and_publish_voltage(self):
        try:
            if self.batterySensor is not None:
                voltage = self.batterySensor.read_voltage()
                self.get_logger().info(f'Aktuelle Batteriespannung: {voltage:.2f} V')

                msg = Battery()
                msg.battery_current = 0.0
                msg.battery_voltage = 0.0
                msg.battery_level = self.__estimate_level(
                    msg.battery_voltage,
                    battery_min=self.battery_low,
                    battery_max=self.battery_full
                )
            else:
                self.get_logger().warn('BatterySensor nicht verfügbar')
                msg = Battery()
                msg.battery_current = 0.0
                msg.battery_voltage = float(random.randrange(1320, 1620)) / 100.0
                msg.battery_level = self.__estimate_level(
                    voltage=msg.battery_voltage,
                    battery_min=self.battery_low,
                    battery_max=self.battery_full
                )

        except Exception as e:
            self.get_logger().error(f'Fehler beim Lesen des Batteriesensors: {e}')
        
        
        self.battery_publisher.publish(msg)



    def __estimate_level(self, voltage: float, battery_min: float, battery_max:float) -> int:
        MIN_VOLTAGE = battery_min if battery_min > 0.0 else 13.1
        MAX_VOLTAGE = battery_max if battery_max > 0.0 else 16.2

        # Begrenzen auf den Spannungsbereich
        clamped = max(min(voltage, MAX_VOLTAGE), MIN_VOLTAGE)

        # Prozentsatz berechnen
        level = ((clamped - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0

        return int(round(level))


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()




# import rclpy
# from rclpy.node import Node
# from .sensors.lidar_sensor import LidarSensor
# from .sensors.battery_sensor import BatterySensor
# from std_msgs.msg import Float32

# """
# Sensor-Node fragt unterschiedliche externe Sensoren ab, verarbeitet diese oder publiziert Ergebnisse weiter

# """
# class SensorNode(Node):
#     def __init__(self):
#         super().__init__('sensor_node')
#         self.get_logger().info('SensorNode gestartet')
#         #lidar_model = self.declare_parameter("lidar_model", "tm30").get_parameter_value().string_value
#         lidar_topic = self.declare_parameter("lidar_topic", "/scan").get_parameter_value().string_value
#         self.lidar = LidarSensor(lidar_topic)

#         # Publisher für Batteriespannung
#         self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)

#         try:
#             self.batterySensor = BatterySensor(self.get_logger(), chan=0, gain=1)
#         except Exception as err:
#             self.get_logger().error(f"ADS1115-Objekt nicht verfügbar => {err}")
#             self.batterySensor = None
                                    
#         self.timer = self.create_timer(10.0, self.read_and_publish_voltage)

#     def read_and_publish_voltage(self):
#         try:
#             if self.batterySensor != None:
#                 voltage = self.batterySensor.read_voltage()

#                 # Konsolenausgabe
#                 self.get_logger().info(f'Aktuelle Batteriespannung: {voltage:.2f} V')

#                 # Publikation
#                 msg = Float32( )
#                 msg.data = voltage
#                 self.voltage_pub.publish(msg)
#             else:
#                 self.get_logger().warn(f'Battery-Sensor aktuell nicht verfügbar')


#         except Exception as e:
#             self.get_logger().error(f'Fehler beim Lesen des Batteriesensors: {e}')

#     def destroy_node(self):
#         self.batterySensor.ads.i2c_device.i2c.unlock()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = SensorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
