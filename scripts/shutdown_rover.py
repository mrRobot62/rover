#!/usr/bin/env python3

import subprocess
import time

# Liste deiner Lifecycle-Nodes
nodes = [
    "lifecycle_node1",
    "lifecycle_node2",
    "odom_node",
    "vision_node",
    "sensor_node",
    "driver_controller_node"
]

def run_lifecycle_cmd(node_name, transition):
    try:
        print(f"🔄 {node_name}: {transition}")
        subprocess.run(
            ["ros2", "lifecycle", "set", f"/{node_name}", transition],
            check=True
        )
    except subprocess.CalledProcessError:
        print(f"⚠️  Fehler bei {node_name}: {transition}")

def main():
    for node in nodes:
        run_lifecycle_cmd(node, "deactivate")
        time.sleep(1)

    for node in nodes:
        run_lifecycle_cmd(node, "shutdown")
        time.sleep(1)

    print("✅ Alle LifecycleNodes heruntergefahren.")

if __name__ == "__main__":
    main()