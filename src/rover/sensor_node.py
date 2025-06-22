import rclpy
from rclpy.node import Node
from .sensors.lidar_sensor import LidarSensor
from .sensors.battery_sensor import BatterySensor
from std_msgs.msg import Float32
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.get_logger().info('SensorNode gestartet')
        #lidar_model = self.declare_parameter("lidar_model", "tm30").get_parameter_value().string_value
        lidar_topic = self.declare_parameter("lidar_topic", "/scan").get_parameter_value().string_value
        self.lidar = LidarSensor(lidar_topic)

        # Publisher für Batteriespannung
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.sensor = BatterySensor(chan=0, gain=1)
        self.timer = self.create_timer(10.0, self.read_and_publish_voltage)

    def read_and_publish_voltage(self):
        try:
            voltage = self.sensor.read_voltage()

            # Konsolenausgabe
            self.get_logger().info(f'Aktuelle Batteriespannung: {voltage:.2f} V')

            # Publikation
            msg = Float32( )
            msg.data = voltage
            self.voltage_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Fehler beim Lesen des Batteriesensors: {e}')

    def destroy_node(self):
        self.sensor.ads.i2c_device.i2c.unlock()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
