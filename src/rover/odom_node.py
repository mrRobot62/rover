# rover/src/rover/odom_node.py

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher

from nav_msgs.msg import Odometry
import math


class OdomNode(LifecycleNode):
    def __init__(self):
        super().__init__('odom_node')
        self.get_logger().info('OdomNode constructed.')

        self.odom_publisher = None
        self.timer = None
        self.counter = 0  # Für Dummy-Odometrie

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_configure()')

        # Parameter deklarieren und ggf. laden
        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        # Publisher vorbereiten
        self.odom_publisher = self.create_lifecycle_publisher(Odometry, self.odom_topic, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_activate()')
        try:
            if self.odom_publisher is not None:
                self.odom_publisher.on_activate(state)
            else:
                self.get_logger().error('odom_publisher ist None!')

            self.timer = self.create_timer(0.1, self.publish_dummy_odom)

        except Exception as e:
            self.get_logger().error(f'Fehler in on_activate(): {e}')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_deactivate()')

        if self.odom_publisher is not None:
            self.odom_publisher.on_deactivate(state)

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_cleanup()')

        if self.odom_publisher is not None:
            self.destroy_publisher(self.odom_publisher)
            self.odom_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_shutdown()')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('[OdomNode] on_error() - Fehlerzustand eingetreten')
        return TransitionCallbackReturn.SUCCESS

    def publish_dummy_odom(self):
        # Dummy-Odometrie zum Testen
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Beispiel: einfache Kreisbewegung simulieren
        angle = 0.1 * self.counter
        radius = 1.0
        msg.pose.pose.position.x = radius * math.cos(angle)
        msg.pose.pose.position.y = radius * math.sin(angle)
        msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        msg.pose.pose.orientation.w = math.cos(angle / 2.0)

        self.odom_publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node

# class OdomNode(Node):
#     def __init__(self):
#         super().__init__('odom_node')
#         self.get_logger().info('OdomNode gestartet')

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
