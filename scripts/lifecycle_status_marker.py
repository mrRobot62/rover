# lifecycle_status_marker.py

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import std_msgs.msg

class LifecycleStatusMarker(Node):
    def __init__(self):
        super().__init__('lifecycle_status_marker')

        self.marker_pub = self.create_publisher(Marker, '/lifecycle_status_marker', 10)
        self.cli = self.create_client(GetState, '/odom_node/get_state')

        self.timer = self.create_timer(1.0, self.publish_status_marker)  # alle 1 Sekunde

    def publish_status_marker(self):
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning('Service /odom_node/get_state nicht verfügbar')
            return

        req = GetState.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lifecycle_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.2
        marker.scale.z = 0.3
        marker.lifetime.sec = 1

        if future.result() is not None:
            label = future.result().current_state.label
            marker.text = f'odom_node: {label}'
            if label == 'active':
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # grün
            elif label == 'inactive':
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # gelb
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # rot
        else:
            marker.text = 'odom_node: ???'
            marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # grau

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = LifecycleStatusMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
