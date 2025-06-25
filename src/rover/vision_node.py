# rover/src/rover/vision_node.py

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher

from sensor_msgs.msg import Image
from std_msgs.msg import String

class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('VisionNode constructed.')

        # Platzhalter für Publisher/Subscriber
        self.image_subscriber = None
        self.result_publisher = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_configure()')

        # Parameter auslesen (z. B. topic name)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        # Publisher vorbereiten (noch nicht aktiv)
        self.odom_publisher = self.create_lifecycle_publisher(Image, self.camera_topic, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_activate()')

        # Subscriber aktivieren
        self.image_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # Publisher aktivieren
        self.result_publisher.on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_deactivate()')

        # Subscriber deaktivieren
        if self.image_subscriber is not None:
            self.destroy_subscription(self.image_subscriber)
            self.image_subscriber = None

        # Publisher deaktivieren
        self.result_publisher.on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_cleanup()')

        # Ressourcen bereinigen
        if self.result_publisher is not None:
            self.destroy_publisher(self.result_publisher)
            self.result_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_shutdown()')
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg: Image):
        self.get_logger().debug('[VisionNode] Received image')
        # Bildverarbeitung hier (Dummy-Logik)
        result = String()
        result.data = 'Erkanntes Objekt: <n/a>'
        self.result_publisher.publish(result)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

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

# class VisionNode(Node):
#     def __init__(self):
#         super().__init__('vision_node')
#         self.get_logger().info('VisionNode gestartet')

# def main(args=None):
#     rclpy.init(args=args)
#     node = VisionNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
