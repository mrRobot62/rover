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

#from rover.hardware.i2c_driver import ESP32RawDriver, ServoDriver, DIGITALPINS
from .hardware.rover_driver import RoverDriver
from .hardware.i2c_driver import SingletonI2CBus, CommandID, SubCommandID, DIGITALPINS
import time

"""
    I2C Node

    kapselt den Zugriff auf die I2C-Schnittstelle und bietet anderen Nodes die Möglichkeit mit 
    I2C Slaves zu kommunizieren.

    Möglichkeit 1: (I2CWrite.msg)
    Subscribe topic /i2c/write. Ein Publisher schreibt einen SCHREIB-Nachricht in dieses Topic
    und wird anschließend vom I2C_node entgegen genommenl, verarbeitet und an den I2C-Slave gesendet

    Möglichkeit 2: (I2CReadResult.msg)
    Liest/Schreibt einer Service-RequestResponse Nachricht. Die RequestDaten enthalten, welcher Slave angesprochen
    wird und andere Details. Der Response Teil ist das Ergebnis vom i2C Slave

    Möglichkeit 3: (I2Cesp32ReadRequest.srv)
    ROS 2 Service zur gezielten Abfrage an den ESP32 stellt und das Ergebnis (Response) versendet
    Typisches Szenario: Abfrage von Servo-Daten über den ESP32

    Möglichkeit 4: (I2CReadRequest.srv)
    ROS 2 Service zur gezielten generische Abfrage eines beliebigen I2C-Slaves
    Rückgabe struktur gilt für alle Slaves gleichermaßen


"""

class I2CNode(LifecycleNode):
    def __init__(self):
        self.node_name = self.__class__.__name__
        super().__init__('i2c_node')
        self.get_logger().info("I2C LifecycleNode instantiated")

        # Parameter auslesen
        self.declare_parameters(
        namespace='',
        parameters=[
            ('i2c_write', '/i2c/write'),
            ('i2c_read', '/i2c/read'),
            ('i2c_timer_update', 0.5),
            ('i2c_timer_periodic_update', 15.0),
            ('i2c_bus_id', 1),
            ('i2c_esp_addr', 0x12),
            ('i2c_ads_addr', 0x48),
            ('i2c_esp32_raise_onerror', True),
            ('i2c_ads_raise_onerror', False),
        ])

        #
        self.i2c_write = self.get_parameter('i2c_write').get_parameter_value().string_value
        self.i2c_read = self.get_parameter('i2c_read').get_parameter_value().string_value
        self.i2c_timer_update = self.get_parameter('i2c_timer_update').get_parameter_value().double_value
        self.i2c_timer_periodic_update = self.get_parameter('i2c_timer_periodic_update').get_parameter_value().double_value
        self.i2c_bus_id = self.get_parameter('i2c_bus_id').get_parameter_value().integer_value
        self.i2c_esp_addr = self.get_parameter('i2c_esp_addr').get_parameter_value().integer_value
        self.i2c_ads_addr = self.get_parameter('i2c_ads_addr').get_parameter_value().integer_value
        self.i2c_esp32_raise_onerror = self.get_parameter('i2c_esp32_raise_onerror').get_parameter_value().bool_value
        self.i2c_ads_raise_onerror = self.get_parameter('i2c_ads_raise_onerror').get_parameter_value().bool_value



        #self.esp_driver = ESP32RawDriver(self.get_logger())
        #self.servo_driver = ServoDriver(self.get_logger())
        self.timer = None

        self.get_logger().info(
        f"""
        I2CNode config:\n\
        --------------------------------
        Topic WRITE:                {self.i2c_write}
        Topic READ:                 {self.i2c_read}
        i2c_timer_update:           {self.i2c_timer_update}
        i2c_timer_periodic_update:  {self.i2c_timer_periodic_update}
        I2C-BUS-ID:                 {self.i2c_bus_id}
        I2C-ESP32-ADDR:             {self.i2c_esp_addr}
        I2C-ESP32-RAISE-ERR:        {self.i2c_esp32_raise_onerror}
        I2C-ADS-ADDR:               {self.i2c_ads_addr}
        I2C-ADS-RAISE-ERR:          {self.i2c_ads_raise_onerror}

        """)
        self.get_logger().info(f"[{self.node_name}] Node im Status unconfigured")
        
    def on_configure(self, state: State):
        self.get_logger().info('I2CNode configured.')
        # Subscriber für WRITE-Befehle
        try:
            #
            # Subscription Möglichkeit 1
            # Typisch für ESP32
            self.get_logger().info(f'Subscribe {self.i2c_write}')
            self.create_subscription(I2CWrite, self.i2c_write, self.handle_write, 10)

            # Service für READ-Anfragen Mögichkeit 2
            self.get_logger().info(f'Create Service {self.i2c_read}')
            self.create_service(I2CReadRequest, self.i2c_read, self.handle_read)

            # Bus-Instanz erhalten
            self.bus = SingletonI2CBus.getBus(self.i2c_bus_id)
            self.get_logger().info(f"I2C-Bus Instanz {self.bus}")

            self.rover_driver = RoverDriver(self.get_logger(), self.bus)

        except Exception as e:
            self.get_logger().error(f'[{self.node_name}] Fehler in on_configure(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())            
        self.get_logger().info(f'[{self.node_name}] Konfigurierung erfolgreich abgeschlossen.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info(f'on_activate()')
        self.periodic_timer = None
        try:
            self.get_logger().info(f'Ping to ESP32...()')
            self.esp_ready = SingletonI2CBus.pingSlave(self.i2c_bus_id, self.i2c_esp_addr)
            if self.esp_ready == False:
                if self.i2c_esp32_raise_onerror:
                    raise IOError(f"EPS32 nicht erreichbar mit {hex(self.i2c_esp_addr)}")
                else:
                    self.get_logger().warn(f"EPS32 nicht erreichbar mit {hex(self.i2c_esp_addr)}")
            else:
                self.get_logger().info(f'EPS32 erreichbar mit  {hex(self.i2c_esp_addr)}')
            
            self.get_logger().info(f'Ping to ADS1115...()')
            self.ads_ready = SingletonI2CBus.pingSlave(self.i2c_bus_id, self.i2c_ads_addr)
            if self.ads_ready == False:
                if self.i2c_ads_raise_onerror:
                    raise IOError(f"ADS1115 nicht erreichbar mit {hex(self.i2c_ads_addr)}")
                else:
                    self.get_logger().warn(f"ADS1115 nicht erreichbar mit {hex(self.i2c_ads_addr)}")
            else:
                self.get_logger().info(f'ADS1115 erreichbar mit  {hex(self.i2c_ads_addr)}')
            # Periodisches Lesen vom ESP32 oder ADS1115
            self.get_logger().info(f'Create PeriodicTimer: {self.i2c_timer_periodic_update}sec')
            self.periodic_timer = self.create_timer(self.i2c_timer_periodic_update, self.periodic_read)


        except Exception as e:
            self.get_logger().error(f'[{self.node_name}] Fehler in on_activate(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())  
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info(f'Aktivierung erfolgreich abgeschlossen.')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info(f'on_deactivate()')
        try:
            if self.timer is not None:
                self.timer.cancel()
                self.destroy_timer(self.timer)
                self.timer = None
        except Exception as e:
            self.get_logger().error(f'[{self.node_name}] Fehler in on_deactivate(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())    

        self.get_logger().info(f'[{self.node_name}] Deaktivierung erfolgreich abgeschlossen.')

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:
            if rclpy.ok():
                self.get_logger().info(f"[{self.node_name}] on_shutdown")
        except Exception as e:
            self.get_logger().error(f'[{self.node_name}] Fehler in on_shutdown(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())    

        self.get_logger().info(f'[{self.node_name}] Shutdown erfolgreich abgeschlossen.')
        return TransitionCallbackReturn.SUCCESS

    def handle_write(self, msg: I2CWrite):
        """
        Handler für I2C-Write-Nachrichten
        """
        if msg.command == CommandID.DIGITAL_WRITE:
            self.esp_driver.digitalWrite(msg.pins, msg.states)
            self.get_logger().debug(f"[{self.node_name}] digital_write => {msg.pins}::{msg.states}")
        elif msg.command == CommandID.SERVO_WRITE:
            #
            # beachten: die parameter reverse_velocity und reverse_steeoring
            # wurden schon vom driver_controller_node in den beide data-werten
            # verrechnet.
            self.rover_driver.set_steeringAndVelocity(
                steering=msg.data[0],
                velocity=msg.data[1]
            )
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

        202507: ADS1115 (Spannung & Strom)
        """
        self.get_logger().info(f"[periodic_read] read I2C Slave....")
        self.
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