# rover/src/rover/odom_node.py

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Odometry
import math


class OdomNode(LifecycleNode):
    def __init__(self):
        super().__init__('odom_node')
        self.get_logger().info('OdomNode constructed.')
        self.callback_group = ReentrantCallbackGroup()

        self.odom_publisher = None
        self.timer = None
        self.counter = 0  # Für Dummy-Odometrie

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_configure()')
        try:
            # Parameter deklarieren und ggf. laden
            self.declare_parameter('odom_topic', '/odom')
            self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

            # Publisher vorbereiten
            self.odom_publisher = self.create_lifecycle_publisher(
                Odometry, self.odom_topic, 10, callback_group=self.callback_group
            )

            self.timer = self.create_timer(0.1, self.publish_dummy_odom, callback_group=self.callback_group)

        except Exception as e:
            self.get_logger().error(f'[OdomNode] Fehler in on_configure(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[OdomNode] on_activate()')
        try:
            if self.odom_publisher is not None:
                result = self.odom_publisher.on_activate(state)
                if result != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error('[OdomNode] odom_publisher konnte nicht aktiviert werden')
                    return TransitionCallbackReturn.FAILURE
            else:
                self.get_logger().error('odom_publisher ist None!')

            self.timer = self.create_timer(0.1, self.publish_dummy_odom)
        except Exception as e:
            self.get_logger().error(f'Fehler in on_activate(): {e}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('[OdomNode] Aktivierung erfolgreich abgeschlossen.')
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
        if rclpy.ok():
            self.get_logger().info(f"[{self.node_name}] on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('[OdomNode] on_error() - Fehlerzustand eingetreten')
        return TransitionCallbackReturn.SUCCESS

    def publish_dummy_odom(self):
        try:
            if self.odom_publisher is None:
                self.get_logger().error("odom_publisher ist None – kein Publish möglich")
                return

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
        except Exception as e:
            self.get_logger().error(f"Fehler im publish_dummy_odom(): {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Node wird beendet...")
        node.destroy_node()
        # ⚠️ Shutdown nur, wenn Kontext nicht schon heruntergefahren!
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
