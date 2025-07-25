#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from lifecycle_msgs.srv import GetState
import sys
import time

def main():
    rclpy.init()
    logger = get_logger('wait_for_lifecycle')

    if len(sys.argv) < 2:
        logger.error("Bitte Node-Namen als Argument übergeben!")
        sys.exit(1)

    node_name = sys.argv[1]

    logger.info(f"Warten auf Service /{node_name}/get_state ...")

    node = rclpy.create_node('wait_for_lifecycle_helper')
    client = node.create_client(GetState, f'/{node_name}/get_state')

    if not client.wait_for_service(timeout_sec=5.0):
        logger.error(f"Service /{node_name}/get_state nicht verfügbar.")
        rclpy.shutdown()
        sys.exit(1)

    request = GetState.Request()
    state = None
    while rclpy.ok():
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if future.done():
            result = future.result()
            state = result.current_state.label
            logger.info(f"[{node.get_name()}]  => STATUS: {node_name}: {state}")
            if state == "active":
                logger.info(f"✅ {node.get_name()} => {node_name} ist aktiv.")
                break

        time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# import sys
# import time
# from rclpy.node import Node
# from lifecycle_msgs.srv import GetState
# from rclpy.logging import get_logger

# def main():
#     if len(sys.argv) < 2:
#         print("❌ Fehler: Node-Name muss als Argument übergeben werden.")
#         sys.exit(1)

#     node_name = sys.argv[1]
#     service_name = f'/{node_name}/get_state'

#     rclpy.init()
#     node = Node(f"wait_for_lifecycle_{node_name.replace('/', '_')}")
#     client = node.create_client(GetState, service_name)

#     print(f"[wait_for_lifecycle] Warten auf Service {service_name} ...")
#     if not client.wait_for_service(timeout_sec=10.0):
#         print("❌ Service nicht verfügbar.")
#         rclpy.shutdown()
#         sys.exit(1)

#     timeout = 30.0
#     start_time = time.time()
#     while time.time() - start_time < timeout and rclpy.ok():
#         req = GetState.Request()
#         future = client.call_async(req)
#         rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
#         if future.done() and future.result():
#             result = future.result()
#             print(f"[STATUS] {node_name}: {result.current_state.label}")
#             if result.current_state.id == 3:  # ACTIVE
#                 print(f"✅ {node_name} ist aktiv.")
#                 rclpy.shutdown()
#                 sys.exit(0)
#         time.sleep(0.5)

#     print(f"❌ Timeout: {node_name} wurde nicht aktiv.")
#     rclpy.shutdown()
#     sys.exit(1)

# if __name__ == '__main__':
#     main()