import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from example_interfaces.msg import Float64MultiArray
from rover_interfaces.msg import I2CWrite, I2CReadResult  # angenommenes Msg-Format
from rover_interfaces.srv import I2CReadRequest

from rover.hardware.i2c_driver import ESP32RawDriver, ServoDriver, DIGITALPINS
import time


class I2CNode(LifecycleNode):
    def __init__(self):
        super().__init__('i2c_node')
        self.get_logger().info("I2C LifecycleNode instantiated")

        # Parameter auslesen
        self.declare_parameters(
        namespace='',
        parameters=[
            ('i2c_write', '/i2c/write'),
            ('i2c_read', '/i2c/read'),
        ])

        self.i2c_write = self.get_parameter('i2c_write').get_parameter_value().string_value
        self.i2c_read = self.get_parameter('i2c_read').get_parameter_value().string_value

        self.esp_driver = ESP32RawDriver(self.get_logger())
        self.servo_driver = ServoDriver(self.get_logger())
        self.timer = None
        self.get_logger().info(
f"""
I2CNode config:\n\
--------------------------------
Topic WRITE:        {self.i2c_write},
Topic READ:         {self.i2c_read}
""")


    def on_configure(self, state: State):
        self.get_logger().info('I2CNode configured.')
        # Subscriber für WRITE-Befehle
        try:
            self.create_subscription(I2CWrite, self.i2c_write, self.handle_write, 10)
            # Service für READ-Anfragen
            self.create_service(I2CReadRequest, self.i2c_read, self.handle_read)
            # Periodisches Lesen vom ESP32 oder ADS1115
            self.timer = self.create_timer(0.5, self.periodic_read)
        except Exception as e:
            self.get_logger().error(f'[I2CNode] Fehler in on_configure(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())            
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('[I2CNode] on_activate()')
        try:
            pass
        except Exception as e:
            self.get_logger().error(f'[I2CNode] Fehler in on_activate(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())            
        self.get_logger().info('[I2CNode] Aktivierung erfolgreich abgeschlossen.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info('[I2CNode] on_deactivate()')

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if rclpy.ok():
            self.get_logger().info(f"[{self.node_name}] on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    def handle_write(self, msg: I2CWrite):
        """
        Handler für I2C-Write-Nachrichten
        """
        if msg.command == "digital_write":
            self.esp_driver.digitalWrite(msg.pins, msg.states)
            self.get_logger().debug(f"[{self.node_name}] digital_write => {msg.pins}::{msg.states}")
        elif msg.command == "servo":
            self.servo_driver.write(cmd=msg.cmd, scmd=msg.subcmd, servo_data=msg.data)
            self.get_logger().debug(f"[{self.node_name}] servo => msg.cmd:{msg.cmd}")
        else:
            self.get_logger().warn(f"Unbekannter Befehl: {msg.command}")

    def handle_read(self, request, response):
        """
        Beispielhafte Antwortfunktion (z. B. für analoge Werte vom ESP32)
        """
        # Hier könnte man per I2C lesen (z. B. ADS1115) und Response befüllen
        values = [0.0, 1.1]  # Dummy
        response.values = values
        return response

    def periodic_read(self):
        """
        Liest regelmäßig Werte von angeschlossenen Geräten.
        """
        # Hier kann z. B. ein ADC gelesen und auf Topic publiziert werden
        pass


def main(args=None):
    rclpy.init(args=args)
    node = I2CNode()

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