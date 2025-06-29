# === lifecycle_node1.py ===
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class LifecycleNode1(LifecycleNode):
    def __init__(self, node_name="lifecycle_node1"):
        super().__init__(node_name)
        self.node_name = node_name
        self.callback_group = ReentrantCallbackGroup()
        self.get_logger().info('LifecycleNode1 constructed')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.node_name}] on_configure")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.node_name}] on_activate")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.node_name}] on_deactivate")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.node_name}] on_cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if rclpy.ok():
            self.get_logger().info(f"[{self.node_name}] on_shutdown")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNode1()
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