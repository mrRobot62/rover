# rover/src/rover/lifecycle_status_marker.py

import rclpy
from rclpy.node import Node
#from rcl_interfaces.msg import State
from lifecycle_msgs.msg import State as LifecycleState
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class LifecycleStatusMarker(Node):
    def __init__(self):
        super().__init__('lifecycle_status_marker')

        # Parameter: Liste der Lifecycle-Nodes, die überwacht werden sollen
        self.declare_parameter('node_names', ['odom_node'])

        self.node_names = self.get_parameter('node_names').get_parameter_value().string_array_value
        self.state_subs = []

        # Publisher für RViz-Marker
        self.marker_pub = self.create_publisher(Marker, 'lifecycle_status_marker', 10)

        for i, name in enumerate(self.node_names):
            topic = f'/{name}/transition_event'
            self.get_logger().info(f'Überwache Lifecycle-Node: {name}')
            sub = self.create_subscription(
                LifecycleState,
                f'/{name}/state',
                lambda msg, node=name, index=i: self.publish_marker(msg, node, index),
                10
            )
            self.state_subs.append(sub)

    def publish_marker(self, msg, node_name, index):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"lifecycle_{node_name}"
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.5 * index
        marker.pose.position.z = 1.5
        marker.scale.z = 0.3
        marker.text = f"{node_name}: {self.state_to_text(msg.label)}"
        marker.color = self.state_to_color(msg.id)
        self.marker_pub.publish(marker)

    def state_to_text(self, state_label: str):
        return state_label

    def state_to_color(self, state_id: int):
        color = ColorRGBA()
        color.a = 1.0
        if state_id == LifecycleState.PRIMARY_STATE_INACTIVE:
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0  # Gelb
        elif state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
            color.g = 1.0  # Grün
        elif state_id == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
            color.r = 1.0  # Rot
        elif state_id == LifecycleState.PRIMARY_STATE_FINALIZED:
            color.b = 1.0  # Blau
        else:
            color.r = 1.0
            color.b = 1.0  # Magenta für unbekannt
        return color

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleStatusMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()