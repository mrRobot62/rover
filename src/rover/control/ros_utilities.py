from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import LifecycleNode

import rclpy
from lifecycle_msgs.msg import State
import time

# State.PRIMARY_STATE_UNKNOWN = 0
# State.PRIMARY_STATE_UNCONFIGURED = 1
# State.PRIMARY_STATE_INACTIVE = 2
# State.PRIMARY_STATE_ACTIVE = 3
# State.PRIMARY_STATE_FINALIZED = 4

LIFECYCLE_STATE_LABELS = {
    State.PRIMARY_STATE_UNKNOWN: "unknown",
    State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
    State.PRIMARY_STATE_INACTIVE: "inactive",
    State.PRIMARY_STATE_ACTIVE: "active",
    State.PRIMARY_STATE_FINALIZED: "finalized",
}

def is_lifecycle_node_active(node: rclpy.node.Node, target_node_name: str, timeout_sec: float = 2.0) -> bool:
    """
    Prüft, ob ein Lifecycle-Node aktiv ist.
    """
    client = node.create_client(GetState, f'/{target_node_name}/get_state')

    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().warn(f"[is_node_active] Service '/{target_node_name}/get_state' nicht erreichbar.")
        return False
    node.get_logger().info(f"f'/{target_node_name}/get_state' verfügbar...")
    
    request = GetState.Request()
    future = client.call_async(request)

    start_time = time.time()
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > timeout_sec:
            node.get_logger().warn(f"[is_node_active] Anfrage an '/{target_node_name}/get_state' fehlgeschlagen (Timeout).")
            return False

    result = future.result()
    if result is None:
        node.get_logger().warn(f"[is_node_active] Future-Ergebnis ist None")
        return False

    state_id = result.current_state.id
    state_label = result.current_state.label
    node.get_logger().info(f"[is_node_active] Zustand von '{target_node_name}': {state_label} ({state_id})")

    return state_id == State.PRIMARY_STATE_ACTIVE

    # client = node.create_client(GetState, f'/{target_node_name}/get_state')

    # if not client.wait_for_service(timeout_sec=timeout_sec):
    #     node.get_logger().warn(f"[is_node_active] Service '/{target_node_name}/get_state' nicht erreichbar.")
    #     return False

    # request = GetState.Request()
    # future = client.call_async(request)
    # rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    # if not future.done():
    #     node.get_logger().warn(f"[is_node_active] Anfrage an '/{target_node_name}/get_state' fehlgeschlagen.")
    #     return False

    # result = future.result()
    # # HUMBLE
    # # state_id = result.state.id
    # # state_label = result.state.label

    # #
    # # JAZZY
    # state_id = result.current_state.id
    # state_label = result.current_state.label

    # node.get_logger().info(f"[is_node_active] Zustand von '{target_node_name}': {state_label} ({state_id})")

    # return state_id == State.PRIMARY_STATE_ACTIVE


# def wait_lifecycle_node_active(node: rclpy.node.Node, target_node_name: str, timeout_sec: float = 2.0, max_retries: int = 10) -> bool:
#     """
#     Wartet, bis der Lifecycle-Node aktiv ist, oder bricht nach max_retries ab.
#     """
#     retry = 0
#     node.get_logger().error(f"[wait_lifecycle_node_active] '{target_node_name}' ....")
#     while retry < max_retries:
#         if is_lifecycle_node_active(node, target_node_name, timeout_sec=timeout_sec):
#             node.get_logger().info(f"[wait_lifecycle_node_active] '{target_node_name}' ist aktiv.")
#             return True

#         node.get_logger().warn(f"[wait_lifecycle_node_active] '{target_node_name}' noch nicht aktiv. Warte erneut... ({retry+1}/{max_retries})")
#         retry += 1
#         time.sleep(1.0)

#     node.get_logger().error(f"[wait_lifecycle_node_active] '{target_node_name}' ist nach {max_retries} Versuchen nicht aktiv geworden.")
#     return False

def wait_lifecycle_node_active_async(node: rclpy.node.Node,
                                     target_node_name: str,
                                     callback,
                                     timeout_sec: float = 2.0,
                                     max_retries: int = 10):
    """
    Nicht-blockierende Version: Überprüft wiederholt per Timer,
    ob der LifecycleNode aktiv ist. Führt Callback aus bei Erfolg oder Misserfolg.

    :param node: ROS Node
    :param target_node_name: Zu überwachender Lifecycle-Node
    :param callback: Funktion mit Parameter: success: bool
    """
    retry = {'count': 0}  # mutable closure

    def check():
        if is_lifecycle_node_active(node, target_node_name, timeout_sec=timeout_sec):
            node.get_logger().info(f"[wait] '{target_node_name}' ist aktiv.")
            timer.cancel()
            callback(True)
        elif retry['count'] >= max_retries:
            node.get_logger().error(f"[wait] '{target_node_name}' wurde nach {max_retries} Versuchen nicht aktiv.")
            timer.cancel()
            callback(False)
        else:
            retry['count'] += 1
            node.get_logger().warn(f"[wait] '{target_node_name}' noch nicht aktiv. Wiederhole... ({retry['count']}/{max_retries})")

    timer = node.create_timer(1.0, check)

def is_node_alive(local_node: Node, target_node_name: str) -> bool:
    """ Prüft ob ein 'normales' Node verfügbar ist. Rückgabe True/False"""
    node_names = local_node.get_node_names()
    return target_node_name in node_names

def delay_once(node, delay_sec: float, callback):
    """
    Erstellt einen einmaligen nicht-blockierenden Delay über ROS-Timer.

    :param node: Der Node, auf dem der Timer registriert wird.
    :param delay_sec: Zeit in Sekunden bis zum Callback.
    :param callback: Funktion, die nach Ablauf der Zeit aufgerufen wird.
    :return: Das Timer-Objekt, das man ggf. abbrechen kann.
    """
    timer = None

    def wrapper():
        nonlocal timer
        timer.cancel()  # sicherheitshalber
        callback()

    timer = node.create_timer(delay_sec, wrapper)
    return timer