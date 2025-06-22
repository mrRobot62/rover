# lifecycle_dashboard.py
# Einfache Weboberfläche zur Anzeige von ROS 2 Lifecycle-Zuständen


# pip install flask
# 
# Start mit ros:    ros2 run rover lifecycle_dashboard.py
# Start direkt :    python3 lifecycle_dashboard.py
#
# Aufruf auf dem PI :           http://localhost:5000/api/lifecycle
# Aufruf über das netzwerk :    http://<ip_deines_roboters>:5000/api/lifecycle
#
# Rückgabe lediglich eine json-Datei den den Status zeigt
# {
#   "/odom_node": "active",
#   "/sensor_node": "inactive",
#   "/drive_controller_node": "unconfigured"
# }

import rclpy
from rclpy.node import Node
from flask import Flask, jsonify
from threading import Thread
from lifecycle_msgs.srv import GetState

app = Flask(__name__)

# Liste der Lifecycle-Nodes, die überwacht werden sollen
LIFECYCLE_NODES = [
    '/odom_node',
    '/sensor_node',
    '/drive_controller_node'
]

# Node zur Kommunikation mit ROS 2 Lifecycle-Services
class LifecycleMonitor(Node):
    def __init__(self):
        super().__init__('lifecycle_monitor')
        self.cli_map = {
            name: self.create_client(GetState, f'{name}/get_state')
            for name in LIFECYCLE_NODES
        }

    def get_states(self):
        states = {}
        for name, client in self.cli_map.items():
            if not client.wait_for_service(timeout_sec=0.5):
                states[name] = 'unreachable'
                continue
            req = GetState.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                state = future.result().current_state.label
                states[name] = state
            else:
                states[name] = 'unknown'
        return states

ros_node = None

@app.route('/api/lifecycle')
def lifecycle():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'ROS node not initialized'}), 500
    return jsonify(ros_node.get_states())

# Flask in einem separaten Thread starten

def run_flask():
    app.run(host='0.0.0.0', port=5000)

def main():
    global ros_node
    rclpy.init()
    ros_node = LifecycleMonitor()
    flask_thread = Thread(target=run_flask, daemon=True)
    flask_thread.start()
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
