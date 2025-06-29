# rover/lifecycle/lifecycle_manager.py

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time


class LifecycleManager(Node):
    def __init__(self, node_names):
        super().__init__('rover_lifecycle_manager')
        self.node_names = node_names
        self.lifecycle_clients = {}

        for name in node_names:
            self.lifecycle_clients[name] = {
                'get_state': self.create_client(GetState, f'{name}/get_state'),
                'change_state': self.create_client(ChangeState, f'{name}/change_state')
            }

        # Start as soon as everything is ready
        self.timer = self.create_timer(5.0, self._startup)
        self.startup_done = False

    def _wait_for_service(self, client, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {client.srv_name} not available.')
            return False
        return True

    def _get_state_label(self, node_name):
        client = self.lifecycle_clients[node_name]['get_state']
        req = GetState.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result():
            state_id = future.result().current_state.id
            state_label = future.result().current_state.label
            self.get_logger().info(f"{node_name}: Aktueller Zustand: {state_label} [{state_id}]")
            return state_label
        self.get_logger().warning(f"{node_name}: Status konnte nicht abgefragt werden.")
        return "unknown"

    def _startup(self):
        if self.startup_done:
            return

        self.get_logger().info('Starte LifecycleNode-Steuerung...')
        time.sleep(3.0)
        success = True

        for node_name in self.node_names:
            clients = self.lifecycle_clients[node_name]

            if not (self._wait_for_service(clients['get_state']) and self._wait_for_service(clients['change_state'])):
                success = False
                continue

            self._get_state_label(node_name)

            # CONFIGURE
            if not self._change_state(node_name, Transition.TRANSITION_CONFIGURE):
                self.get_logger().info(f"[{node_name}] - TRANSITION_CONFIGURE nicht erfolgreich durchgeführt")
                success = False
                continue

            time.sleep(1.0)
            self._get_state_label(node_name)

            # ACTIVATE
            if not self._change_state(node_name, Transition.TRANSITION_ACTIVATE):
                self.get_logger().info(f"[{node_name}] - TRANSITION_ACTIVATE nicht erfolgreich durchgeführt")
                success = False

            self._get_state_label(node_name)
            time.sleep(1.0)

        if success:
            self.get_logger().info('Alle LifecycleNodes erfolgreich aktiviert.')
        else:
            self.get_logger().error('Einige LifecycleNodes konnten nicht aktiviert werden.')

        self.startup_done = True
        self.destroy_timer(self.timer)

    def _change_state(self, node_name, transition_id):
        client = self.lifecycle_clients[node_name]['change_state']
        req = ChangeState.Request()
        req.transition.id = transition_id
        self.get_logger().info(f"[{node_name}] - _change_state (1) {req}")

        future = client.call_async(req)

        # Warten bis abgeschlossen (manuelles Spin statt spin_until_future_complete!)
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10.0:
                self.get_logger().error(f"{node_name}: Timeout beim Warten auf Future für Transition {transition_id}")
                return False

        self.get_logger().info(f"[{node_name}] - _change_state (2)")

        result = future.result()
        self.get_logger().info(f"[{node_name}] - _change_state (3) {result}")
        if result is None or not result.success:
            self.get_logger().error(f"{node_name}: Transition {transition_id} fehlgeschlagen (no result oder success=False).")
            return False

        self.get_logger().info(f"{node_name}: Transition {transition_id} erfolgreich.")
        return True


def main(args=None):
    rclpy.init(args=args)

    # 💡 Liste deiner Lifecycle-Nodes hier:
    nodes_to_manage = [
        '/lifecycle_node1',
        '/lifecycle_node2'
        # '/odom_node',
        # '/sensor_node',
        # '/vision_node'
    ]

    manager = LifecycleManager(nodes_to_manage)
    executor = MultiThreadedExecutor()
    executor.add_node(manager)

    try:
        executor.spin()
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

