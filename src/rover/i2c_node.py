import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

from rover_interfaces.msg import I2CWrite, I2CRead, BatteryRaw  # angenommenes Msg-Format
from rover_interfaces.srv import I2CESP32Communication, I2CReadRequest

from .control.ESP32CommandsV1 import CommandID, SubCommandID, ESP32PINS, SERVICE_RESPONSE, ESP32_RESPONSE
from .control.utilities import Utilities
from .hardware.ads1115_driver import ADS1115Driver
from .hardware.acs712_driver import ACS712Driver
from .hardware.i2c_driver import I2CBus
import struct

#from rover.hardware.i2c_driver import ESP32RawDriver, ServoDriver, DIGITALPINS
#from .hardware.rover_driver import RoverDriver
#from .hardware.i2c_driver import CommandID, SubCommandID, ESP32PINS
import time

"""
    I2C Node

    kapselt den Zugriff auf die I2C-Schnittstelle und bietet anderen Nodes die Möglichkeit mit 
    I2C Slaves zu kommunizieren.

    Möglichkeit 1: (I2CWrite.msg)
    Subscribe topic /i2c/write. Ein Publisher schreibt einen SCHREIB-Nachricht in dieses Topic
    und wird anschließend vom I2C_node entgegen genommenl, verarbeitet und an den I2C-Slave gesendet

    Möglichkeit 2: (I2CRead.msg)
    Liest/Schreibt einer Service-RequestResponse Nachricht. Die RequestDaten enthalten, welcher Slave angesprochen
    wird und andere Details. Der Response Teil ist das Ergebnis vom i2C Slave

    Möglichkeit 3: (I2CESP32Communication) vorher (I2Cesp32ReadRequest.srv)
    ROS 2 Service der gezieht nur den ESP32 anspricht. Der Service kann grundsätzlich auch
    Daten vom ESP32 empfangen (Response). Responsedaten bestehen aus eine Array fvalues[] und ivalues[]. Der Request besteht Command/Subcommand & Daten

    Möglichkeit 4: (I2CReadRequest.srv)
    ROS 2 Service zur gezielten generische Abfrage eines beliebigen I2C-Slaves
    Rückgabe struktur gilt für alle Slaves gleichermaßen


"""

class I2CNode(LifecycleNode):
    def __init__(self):
        self.node_name = self.__class__.__name__
        super().__init__('i2c_node')

        self.declare_parameter("log_level", "INFO")  # Default als Fallback
        level_str = self.get_parameter("log_level").get_parameter_value().string_value
        from rclpy.logging import LoggingSeverity
        log_level = getattr(LoggingSeverity, level_str.upper(), LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)
        self.get_logger().info("I2C LifecycleNode instantiated")

        ct_bus1 = Utilities.get_common_topic('i2c_bus1', 1, logger=self.get_logger())
        ct_bus2 = Utilities.get_common_topic('i2c_bus2', 2, logger=self.get_logger())
        ct_i2c_esp_adr = Utilities.get_common_topic('i2c_esp_adr', 0x12, logger=self.get_logger())
        ct_i2c_ads_adr = Utilities.get_common_topic('i2c_ads_adr', 0x48, logger=self.get_logger())
        ct_t_write = Utilities.get_common_topic('i2c_write_topic','/i2c/write', logger=self.get_logger())
        ct_t_read = Utilities.get_common_topic('i2c_read_topic','/i2c/read', logger=self.get_logger())
        ct_t_ads_raw = Utilities.get_common_topic('topic_battery_raw','/battery/ads_raw', logger=self.get_logger())
        ct_srv_esp = Utilities.get_common_topic('i2c_esp_command_srv','/i2c/esp32_command', logger=self.get_logger())
        ct_srv_read = Utilities.get_common_topic('i2c_read_request_srv','/i2c/read_request', logger=self.get_logger())

        # Parameter auslesen
        self.declare_parameters(
        namespace='',
        parameters=[
            # Common config
            ('i2c_bus_id', ct_bus1),
            ('i2c_bus_id2', ct_bus2),
            ('i2c_esp_addr',ct_i2c_esp_adr),
            ('i2c_ads_addr', ct_i2c_ads_adr),
            ('i2c_write_topic', ct_t_write),
            ('i2c_read_topic', ct_t_read),
            ('i2c_esp_command_srv', ct_srv_esp),
            ('i2c_ads_raw_topic', ct_t_ads_raw),
            ('i2c_read_request_srv', ct_srv_read),
 
            # I2C_node config
            ('i2c_timer_update', 0.5),
            ('ads1115_sample_rate_hz', 1),    # 1hz = 1xsek
            ('acs712_zero_offset', 2.5),
            ('acs712_sensitivity', 0.185),
            ('acs712_on_channel', 1),
            ('i2c_esp32_raise_onerror', True),
            ('i2c_ads_raise_onerror', False),
        ])

        #
        # Kommunikation
        self.i2c_write_topic = self.get_parameter('i2c_write_topic').get_parameter_value().string_value
        self.i2c_read_topic = self.get_parameter('i2c_read_topic').get_parameter_value().string_value
        self.i2c_esp_command_srv = self.get_parameter('i2c_esp_command_srv').get_parameter_value().string_value
        self.i2c_request_request_srv = self.get_parameter('i2c_read_request_srv').get_parameter_value().string_value

        # Konfigurationen
        self.i2c_timer_update = self.get_parameter('i2c_timer_update').get_parameter_value().double_value
        self.i2c_bus_id = self.get_parameter('i2c_bus_id').get_parameter_value().integer_value
        self.i2c_bus_id2 = self.get_parameter('i2c_bus_id2').get_parameter_value().integer_value
        self.i2c_esp_addr = self.get_parameter('i2c_esp_addr').get_parameter_value().integer_value
        self.i2c_ads_addr = self.get_parameter('i2c_ads_addr').get_parameter_value().integer_value
        self.i2c_esp32_raise_onerror = self.get_parameter('i2c_esp32_raise_onerror').get_parameter_value().bool_value
        self.i2c_ads_raise_onerror = self.get_parameter('i2c_ads_raise_onerror').get_parameter_value().bool_value
        self.i2c_ads_raw_topic = self.get_parameter('i2c_ads_raw_topic').get_parameter_value().string_value

        self.ads1115_sample_rate_hz = self.get_parameter('ads1115_sample_rate_hz').get_parameter_value().integer_value
        self.acs712_zero_offset = self.get_parameter('acs712_zero_offset').get_parameter_value().double_value
        self.acs712_sensitivity = self.get_parameter('acs712_sensitivity').get_parameter_value().double_value
        self.acs712_on_channel = self.get_parameter('acs712_on_channel').get_parameter_value().integer_value


        #self.esp_driver = ESP32RawDriver(self.get_logger())
        #self.servo_driver = ServoDriver(self.get_logger())
        self.timer = None

        self.get_logger().info(
        f"""
        I2CNode config:\n\
        ---------------------------------------------------
        COMMON --------------------------------------------
        TOPIC WRITE:                {self.i2c_write_topic}
        TOPIC READ:                 {self.i2c_read_topic}
        TOPIC BatteryRaw:           {self.i2c_ads_raw_topic}
        SERIVCE ESP32:              {self.i2c_esp_command_srv}
        SERIVCE REQUEST_RESPONSE:   {self.i2c_request_request_srv}
        I2C-BUS-ID1:                {self.i2c_bus_id}
        I2C-BUS-ID2:                {self.i2c_bus_id2}
        I2C-ESP32-ADDR:             {self.i2c_esp_addr} / {hex(self.i2c_esp_addr)}
        I2C-ADS-ADDR:               {self.i2c_ads_addr} / {hex(self.i2c_ads_addr)}

        I2C-NODE -----------------------------------------
        i2c_timer_update:           {self.i2c_timer_update}
        I2C-ESP32-RAISE-ERR:        {self.i2c_esp32_raise_onerror}
        I2C-ADS-RAISE-ERR:          {self.i2c_ads_raise_onerror}
        I2C-ADS-SAMPLE-RATE-HZ      {self.ads1115_sample_rate_hz}
        """)
        self.get_logger().info(f"[{self.node_name}] Node im Status unconfigured")
        
    def on_configure(self, state: State):
        # Subscriber für WRITE-Befehle
        try:
            #
            # Subscription Möglichkeit 1
            self.get_logger().info(f'Topic Subscribe {self.i2c_write_topic}')
            self.create_subscription(
                I2CWrite,                       # Message Type
                self.i2c_write_topic,           # Topic-Name
                self.handle_write_msg,          # callback funktion
                10                              # 
            )

            #
            # Subscription Möglichkeit 2
            self.get_logger().info(f'Topic Subscribe {self.i2c_read_topic}')
            self.create_subscription(
                I2CWrite, 
                self.i2c_read_topic, 
                self.handle_read_msg, 10)

            # Möglichkeit 3
            # Service für READ-Anfragen Mögichkeit 2
            self.get_logger().info(f'Service Create {self.i2c_esp_command_srv}')
            self.create_service(
                I2CESP32Communication,          # Serivce Protokoll-Type
                self.i2c_esp_command_srv,       # Service Protokoll
                self.handle_esp32_command       # callback
            )

            # Möglichkeit 4
            # Service für READ-Anfragen Mögichkeit 2
            self.get_logger().info(f'Service Create {self.i2c_request_request_srv}')
            self.create_service(
                I2CReadRequest,                 # Serivce Protokoll-Type
                self.i2c_request_request_srv,   # Service Protokoll
                self.handle_request_response    # callback
            )

            #
            # Instanzierung des I2C-Busses
            self.bus = I2CBus.getBus(self.i2c_bus_id)
            self.get_logger().info(f"I2C-Bus Instanz {self.bus}")

            #
            # ADS1115Driver nutzen
            self.ads = ADS1115Driver(
                bus=self.bus, 
                logger=self.get_logger(),
                slave_address=self.i2c_ads_addr, 
                gain=0
            )
            self.get_logger().info(f"ADS1115Driver {self.ads} ready")

            #
            # Strom-Messer am ADS1115 
            self.acs712 = ACS712Driver(
                ads_driver=self.ads,
                channel=self.acs712_on_channel,
                zero_offset=self.acs712_zero_offset,
                sensitivity=self.acs712_sensitivity
            )
            self.get_logger().info(f"ACS712Driver {self.acs712} ready")
            self.get_logger().info(f'[{self.node_name}] Konfigurierung erfolgreich abgeschlossen.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'[{self.node_name}] Fehler in on_configure(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())            
            return TransitionCallbackReturn.FAILURE

        
    def on_activate(self, state: State):
        self.get_logger().info(f'on_activate() - started')
        self.periodic_timer = None
        try:
            self.get_logger().info(f'Ping to ESP32...()')
            self.esp_ready = I2CBus.pingSlave(self.i2c_bus_id, self.i2c_esp_addr)
            if self.esp_ready == False:
                if self.i2c_esp32_raise_onerror:
                    raise IOError(f"EPS32 nicht erreichbar mit {hex(self.i2c_esp_addr)}")
                else:
                    self.get_logger().warn(f"\tEPS32 nicht erreichbar mit {hex(self.i2c_esp_addr)}")
            else:
                self.get_logger().info(f'\tEPS32 erreichbar mit  {hex(self.i2c_esp_addr)}')
            
            self.get_logger().info(f'Ping to ADS1115...()')
            self.ads_ready = I2CBus.pingSlave(self.i2c_bus_id, self.i2c_ads_addr)
            if self.ads_ready == False:
                if self.i2c_ads_raise_onerror:
                    raise IOError(f"ADS1115 nicht erreichbar mit {hex(self.i2c_ads_addr)}")
                else:
                    self.get_logger().warn(f"\tåADS1115 nicht erreichbar mit {hex(self.i2c_ads_addr)}")
            else:
                self.get_logger().info(f'\tADS1115 erreichbar mit  {hex(self.i2c_ads_addr)}')

            #
            # BatteryPublisher konfigurieren
            self.ads1115_sample_rate_ms = 1 / self.ads1115_sample_rate_hz
            self.get_logger().info(f'ADS1115 Publisher {(self.ads1115_sample_rate_ms/1000)}ms in {self.i2c_ads_raw_topic}')
            self.pub_batt = self.create_publisher(BatteryRaw, self.i2c_ads_raw_topic, 10)
            self.create_timer(self.ads1115_sample_rate_ms, self.read_ads1115_periodically)

            #
            # IMU Publisher konfigurieren

            #
            # OTHER Publisher konfigurieren



        except Exception as e:
            self.get_logger().error(f'[{self.node_name}] Fehler in on_activate(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())  
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info(f'[{self.node_name}]  erfolgreich aktiviert')
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

    def handle_write_msg(self, msg: I2CWrite):
        """
        Verarbeitet eine WRITE-Anforderung an einen I2C-Slave
        """
        # if msg.command == CommandID.DIGITAL_WRITE:
        #     self.esp_driver.digitalWrite(msg.pins, msg.states)
        #     self.get_logger().debug(f"[{self.node_name}] digital_write => {msg.pins}::{msg.states}")
        # elif msg.command == CommandID.SERVO_WRITE:
        #     #
        #     # beachten: die parameter reverse_velocity und reverse_steeoring
        #     # wurden schon vom driver_controller_node in den beide data-werten
        #     # verrechnet.
        #     self.rover_driver.set_steeringAndVelocity(
        #         steering=msg.data[0],
        #         velocity=msg.data[1]
        #     )
        # else:
        #     self.get_logger().warn(f"Unbekannter Befehl: {msg.command}")
        pass

    def handle_read_msg(self, msg: I2CRead):
        """ 
        Verarbeitet eine READ-Anforderung an einen I2C-Slave
        """
        pass

    def handle_esp32_command(self, request, response):
        """
        Verarbeitet alle ESP32 Commands. Hauptsächlich für die Steuerung der Dynamixel-Servos
        ESP32 antwortet nur bei bestimmten Command/SubCommands. Da der Service aber grundsätzlich einen Response erwartet
        wird bei Bedarf das Response-Ergebnis mit Defaults belegt.

        __send_esp32_packet() senden tatsächlich die Daten
        """
        if request.device != "ESP32":
            self.get_logger().warn("Unbekanntes Zielgerät: " + request.device)
            response.ivalues = [-1]  # Fehlercode
            return response
        try:
            self.get_logger().info(f"--S1")
            if request.command == CommandID.SERVO_WRITE.value:
                if request.subcommand == SubCommandID.SCMD_SERVO_SPEED_POSITION.value:
                    self.get_logger().info(f"--S2")
                    steering = request.fvalues[0]
                    velocity = request.fvalues[1]
                    steering = Utilities.clamp(steering, -1.0, +1.0)
                    velocity = Utilities.clamp(velocity, -1.0, +1.0)
                    self.get_logger().info(f"--S3")
                    #
                    # Die Float-Daten müssen umgewandelt werden in einen
                    # Wertebereich von 0-65535
                    data = self.__dataModulo(request.fvalues, factor=100)
                    return self.__send_esp32_packet(
                        slave_address=self.i2c_esp_addr,
                        cmd=request.command,
                        scmd=request.subcommand,
                        #
                        # Float-Values müssen in Integer umgewandelt werden
                        data_vals = data,
                        response=response
                    )
                else:
                    self.get_logger().warn(f"ESP32-SubCommand '{request.subcommand.value}' unbekannt")
                    response.ivalues = [ESP32_RESPONSE.UNKNOWN_SCMD]    
            else:
                self.get_logger().warn(f"ESP32-Command '{request.command.value}' unbekannt")
                response.ivalues = [ESP32_RESPONSE.UNKNOWN_CMD]    

        except Exception as e:
            self.get_logger().error(f"I2C-Fehler :{e}")
            response.ivalues[0] = SERVICE_RESPONSE.I2C_ERROR

        return response

    def handle_request_response(self, request, response):
        """
        Verarbeitet allgemeinen Datenaustausch mit einem I2C-Slave und erwartet ein Response zurück

        """
        pass

    def read_ads1115_periodically(self):
        """ 
            Periodisches Lesen des ADS1115. 
        """
        try:
            msg = BatteryRaw()
            msg.channel_0 = round(self.ads.scaled_voltage(0, 25.0, 5.0),3)
            msg.channel_1 = round(self.acs712.read_current(),3)      # default 10messungen = 10ms delay
            msg.channel_2 = round(self.ads.read_voltage(2),3)
            msg.channel_3 = round(self.ads.read_voltage(3),3)
            self.pub_batt.publish(msg)
            self.get_logger().info(f"Publish BatteryRaw: {msg}")
        except Exception as e:
            self.get_logger().error(f"ADS1115 Fehler: {e}")

    
    # ---------------------------------------------------------------------------------------
    # private methoden
    # ---------------------------------------------------------------------------------------
    def __send_esp32_packet(self, slave_address:int, cmd:int, scmd:int, data_vals, response, reserved:int=0b0, flags:int=0b0):
        """
        Versenden jetzt tatsächliche ein Datenpaket an den ESP32
        """
        data_vals = data_vals[:5] + [0] * (5 - len(data_vals))
        header = 0xFEEF
    
        # Paketstruktur gemäß deinem Protokoll
        try:
            #self.get_logger().info(f"[__send_esp32_packet] -- 1")
            payload = struct.pack("<HBBB5HB",
                                header,
                                cmd,
                                scmd,
                                flags,
                                *data_vals,
                                reserved)
            #self.get_logger().info(f"[__send_esp32_packet] -- 2")
            crc = self.__crc8(payload)
            packet = payload + bytes([crc])
            #self.get_logger().info(f"[__send_esp32_packet] -- 3")

            # Umwandeln in Liste von ints für write_i2c_block_data
            packet_list = list(packet)
            #self.get_logger().info(f"[__send_esp32_packet] -- 4 {packet_list}")
            self.bus.write_i2c_block_data(slave_address, 0x00, packet_list)
            self.get_logger().info(f"[__send_esp32_packet] write_i2c_block_data done")
        except Exception as e:
            self.get_logger().error(f"I2C-Error: {e}")
            response.ivalues = [SERVICE_RESPONSE.I2C_ERROR]

        return response            

    def __dataModulo(self, data_vals, factor=100):
        """
        Skaliert Float-Werte in einen Bereich von uint16_t (0–65535), unter Verwendung von Modulo-Arithmetik.
        Wird z. B. benötigt, um negative Geschwindigkeits- oder Lenkwinkelwerte als unsigned zu übertragen.

        @param data_vals ist ein Array von Float-Werten
        @param factor default 100 = Multiplikator
        @return array der neu berechneten Werte
        """
        return [int(val * factor) % 65536 for val in data_vals]

    def __crc8(self, data: bytes) -> int:
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
                crc &= 0xFF
        return crc

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