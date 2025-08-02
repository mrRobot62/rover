# obstacle_avoidance_node.py
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .control.utilities import Utilities
from .control.ros_utilities import *
from rover_interfaces.msg import LEDMessage
from .control.led_pattern import LEDPattern

import os
import math

class ObstacleAvoidanceNode(LifecycleNode):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.pid = os.getpid()

        # Parameter deklarieren
        self.declare_parameter("log_level", "INFO")  # Default als Fallback
        level_str = self.get_parameter("log_level").get_parameter_value().string_value
        from rclpy.logging import LoggingSeverity
        log_level = getattr(LoggingSeverity, level_str.upper(), LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

        self.get_logger().info(f"{self.node_name} instantiated")

        ct_tled = Utilities.get_common_topic('topic_led', '/led', logger=self.get_logger())
        ct_bat = Utilities.get_common_topic('topic_battery', '/battery', logger=self.get_logger())
        ct_cmd_vel = Utilities.get_common_topic('cmd_vel_topic', '/joy', logger=self.get_logger())

        self.declare_parameters(
            namespace='',
            parameters=[
                # Common config
                ('topic_led', ct_tled),
                ('topic_battery', ct_bat),
                ('cmd_vel_topic', ct_cmd_vel),
    
                # Node config
                ('scan_topic', 'auto_cmd_vel'),
                ('angle_front', 45.0),
                ('obstacle_distance', 0.5),
                ('forward_speed', 0.2),
                ('turn_speed', 0.4),
            ]
        )

        self._subscriber = None
        self._publisher = None
        self.node_active = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('🔧 on_configure: Initialisiere Parameter und Publisher...')

        # Parameter laden
        self.topic_battery = self.get_parameter('topic_battery').get_parameter_value().string_value
        self.topic_led = self.get_parameter('topic_led').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.angle_front = math.radians(self.get_parameter('angle_front').get_parameter_value().double_value)
        self.obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value

        # Publisher vorbereiten (noch nicht aktiv)
        qos = QoSProfile(depth=10)
        self._publisher = self.create_publisher(Twist, self.cmd_vel_topic, qos)

        # Subscriber vorbereiten
        self._subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.lidar_callback,
            qos
        )

        self.get_logger().info('✅ on_configure abgeschlossen.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('🚀 on_activate: Node ist jetzt aktiv.')
        self.node_active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('⏸ on_deactivate: Node wird pausiert.')
        self.node_active = False
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('🧹 on_cleanup: Ressourcen werden freigegeben.')
        self.destroy_publisher(self._publisher)
        self.destroy_subscription(self._subscriber)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('🛑 on_shutdown: Node wird heruntergefahren.')
        return TransitionCallbackReturn.SUCCESS

    def lidar_callback(self, msg: LaserScan):
        if not self.node_active:
            return

        # Bereich von -angle_front bis +angle_front um 0°
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)

        center_index = int((0.0 - angle_min) / angle_increment)
        angle_range = int(self.angle_front / angle_increment)

        start_index = max(0, center_index - angle_range)
        end_index = min(num_readings, center_index + angle_range)

        # Prüfe, ob Hindernis im Frontbereich ist
        front_ranges = msg.ranges[start_index:end_index]
        obstacle_detected = any(
            0.05 < r < self.obstacle_distance for r in front_ranges
        )

        twist = Twist()
        if obstacle_detected:
            # Hindernis erkannt → drehen
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info('⚠️ Hindernis erkannt – Ausweichen!')
        else:
            # Kein Hindernis → vorwärts
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            self.get_logger().debug('✅ Kein Hindernis – Geradeaus fahren.')

        self._publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
