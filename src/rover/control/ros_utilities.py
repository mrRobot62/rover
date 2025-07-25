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



import rclpy
from rclpy.node import Node
from typing import Callable

class Ros2Delay:
    def __init__(self, node: Node, s: float, callback: Callable):
        """
        Delay-Objekt auf Basis von rclpy.Timer.

        Beispiel:
        def on_delay_done():
            self.get_logger().info("5 Sekunden sind um!")

        self.delay = Ros2Delay(self, 5.0, on_delay_done)
        self.delay.start()

        :param node: ROS2-Node, über den der Timer erstellt wird
        :param s: Verzögerung in Sekunden
        :param callback: Funktion, die nach Ablauf aufgerufen wird
        """
        self.node = node
        self.s = s
        self.callback = callback
        self._timer = None

    def start(self):
        """Startet die Verzögerung."""
        self.cancel()
        self._timer = self.node.create_timer(self.s, self._handle_timeout)

    def _handle_timeout(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None
        if self.callback:
            self.callback()

    def cancel(self):
        """Bricht laufenden Timer ab (wenn vorhanden)."""
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def is_running(self):
        """Gibt zurück, ob ein Delay läuft."""
        return self._timer is not None