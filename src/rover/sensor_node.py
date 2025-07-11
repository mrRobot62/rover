
# rover/src/rover/sensor_node.py

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecyclePublisher

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu 

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from .sensors.lidar_sensor import LidarSensor
from .sensors.battery_sensor import BatterySensor, BatteryStatus, PowerStatus
from .control.led_pattern import LEDPattern
from rover_interfaces.msg import Battery, LEDMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor

import random

class SensorNode(LifecycleNode):

    battery_levels = [
        (100, LEDPattern.BATTERY_100.value),
        (90, LEDPattern.BATTERY_90.value),
        (80, LEDPattern.BATTERY_80.value),
        (70, LEDPattern.BATTERY_70.value),
        (60, LEDPattern.BATTERY_60.value),
        (50, LEDPattern.BATTERY_50.value),
        (40, LEDPattern.BATTERY_40.value),
        (30, LEDPattern.BATTERY_30.value),
        (20, LEDPattern.BATTERY_20.value),
        (10, LEDPattern.BATTERY_10.value),
        (0,  LEDPattern.BATTERY_0.value),
    ]

    def __init__(self):
        self.node_name = self.__class__.__name__
        super().__init__(self.node_name)
        self.lidar = None
        self.batterySensor = None
        self.timer = None
        self.timer2 = None
        self.battery_publisher = None
        self.imu_publisher = None
        self.callback_group = ReentrantCallbackGroup()


        self.get_logger().info(f"[{self.node_name}] Node im Status unconfigured")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.get_logger().info('[SensorNode] on_configure()')

            # Parameter auslesen
            lidar_topic = self.declare_parameter("lidar_topic", "/scan").get_parameter_value().string_value

            # Initialisierung Lidar (z. B. für Softkopplung)
            self.lidar = LidarSensor(lidar_topic)

            # BatterySensor initialisieren
            self.declare_parameters(
                namespace='',
                parameters = [
                    # Common Parameter
                    ('topic_led','/led'),
                    ('topic_battery','/battery'),
                    ('topic_imu','/imu'),
                    ('i2c_bus1',1),
                    ('i2c_esp_adr',0x12),
                    ('i2c_ads_adr', 0x48),

                    # Node Parameter
                    ('battery_sensor_active', True),
                    ('battery_critical', 1.0),
                    ('battery_low', 1.0),
                    ('battery_full', 1.0),
                    ('battery_voltage_channel', 0),
                    ('battery_current_channel', 1),
                    ('battery_i2c_bus_id', 1),
                    ('voltage_max_in', 25.0),
                    ('voltage_max_out', 5.0),
                    ('ads1115_gain', 0),
                    ('acs712_type', 5),
                    ('acs712_vdd', 5.0),
                    ('imu_sensor_active', False),
                ])

            # Common Parameter
            self.i2c_bus1 = self.get_parameter('i2c_bus1').get_parameter_value().integer_value
            self.i2c_esp_adr = self.get_parameter('i2c_ads_adr').get_parameter_value().integer_value
            self.i2c_ads_adr = self.get_parameter('i2c_ads_adr').get_parameter_value().integer_value
            self.topic_battery = self.get_parameter('topic_battery').get_parameter_value().string_value
            self.topic_led = self.get_parameter('topic_led').get_parameter_value().string_value
            self.topic_imu = self.get_parameter('topic_imu').get_parameter_value().string_value

            # Node Parameter
            self.battery_sensor_active = self.get_parameter('battery_sensor_active').get_parameter_value().bool_value
            self.battery_critical = self.get_parameter('battery_critical').get_parameter_value().double_value
            self.battery_low = self.get_parameter('battery_low').get_parameter_value().double_value
            self.battery_full = self.get_parameter('battery_full').get_parameter_value().double_value
            self.voltage_max_in = self.get_parameter('voltage_max_in').get_parameter_value().double_value
            self.voltage_max_out = self.get_parameter('voltage_max_out').get_parameter_value().double_value
            self.battery_voltage_channel = self.get_parameter('battery_voltage_channel').get_parameter_value().integer_value
            self.battery_current_channel = self.get_parameter('battery_current_channel').get_parameter_value().integer_value
            self.ads1115_gain = self.get_parameter('ads1115_gain').get_parameter_value().integer_value

            self.acs712_type = self.get_parameter('acs712_type').get_parameter_value().integer_value
            self.acs712_vdd = self.get_parameter('acs712_vdd').get_parameter_value().double_value

            self.imu_sensor_active = self.get_parameter('imu_sensor_active').get_parameter_value().bool_value
            # Sensoren installieren
            try:
                if self.battery_sensor_active:
                    self.batterySensor = BatterySensor(
                        logger=self.get_logger(),
                        i2c_bus_id=self.i2c_bus1,
                        batMin=self.battery_low,
                        batMax=self.battery_full,
                        voltMaxIn=self.voltage_max_in,
                        voltMaxOut=self.voltage_max_out,
                        batVCh=self.battery_voltage_channel,
                        batCCh=self.battery_current_channel,
                        acs712_type=self.acs712_type,
                        acs712_vdd=self.acs712_vdd,
                        i2c_slave_address=self.i2c_ads_adr,
                        gain=self.ads1115_gain
                    )
                    self.get_logger().warn(f"[self.node_name] => BatterySensor ready")
                else:
                    self.get_logger().warn(f"BatterySensor deaktiviert")
                    self.batterySensor = None
            except Exception as err:
                self.get_logger().error(f"BatterySensor konnte nicht initialisiert werden: {err}")
                self.batterySensor = None
                import traceback
                self.get_logger().error(traceback.format_exc())
                return TransitionCallbackReturn.FAILURE

            try:
                if self.imu_sensor_active:
                    self.get_logger().warn(f"IMU aktuell nicht implementiert")
                    #self.get_logger().warn(f"[self.node_name] => IMUSensor ready")

                else:
                    self.get_logger().warn(f"IMU deaktiviert")
                    self.imuSensor = None
            except Exception as err:
                self.get_logger().error(f"IMUSensor konnte nicht initialisiert werden: {err}")
                self.imuSensor = None
                return TransitionCallbackReturn.FAILURE
                  
            #
            # PUBLISHER aktivieren

            # Allgemeine Topics publishen
            self.led_publisher = self.create_publisher(LEDMessage, self.topic_led, 10)

            # Sensor spezifische Topics publishen
            self.battery_publisher = self.create_lifecycle_publisher(Battery, self.topic_battery, 10)
            self.imu_publisher = self.create_lifecycle_publisher(Imu, self.topic_imu, 10)

            self.get_logger().info(
            f"""
            SensorNode config:\n\
            Publish-Topics
            --------------------------------------------
            LED:                                {self.topic_led}
            BATTERY:                            {self.topic_battery}
            IMU:                                {self.topic_imu}
            --------------------------------------------
            BATTERY-SENSOR
                battery_sensor_active:          {self.battery_sensor_active}
                battery_critical:               {self.battery_critical}
                battery_low:                    {self.battery_low}
                battery_full:                   {self.battery_full}
                battery_voltage_channel:        {self.battery_voltage_channel}
                battery_current_channel:        {self.battery_current_channel}

            IMU-Sensor
                battery_sensor_active:          {self.imu_sensor_active}
            """)

            self.get_logger().info('on_configure() abgeschlossen')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Fehler in on_configure(): {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate()')

        # Publisher aktivieren
        self.battery_publisher.on_activate(state)
        self.get_logger().info(f"battery_publisher aktiviert")
        self.imu_publisher.on_activate(state)
        self.get_logger().info(f"imu_publisher aktiviert")

        # Timer starten
        self.timer = self.create_timer(10.0, self.publish_battery_state)
        self.timer2 = self.create_timer(2.0, self.publish_imu_state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate()')

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        
        if self.timer2 is not None:
            self.timer2.cancel()
            self.destroy_timer(self.timer2)
            self.timer2 = None

        self.battery_publisher.on_deactivate(state)
        self.imu_publisher.on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_cleanup()')

        # Publisher zerstören
        if self.battery_publisher is not None:
            self.destroy_publisher(self.battery_publisher)
            self.battery_publisher = None

        # BatterySensor aufräumen
        if self.batterySensor is not None:
            try:
                self.batterySensor.ads.i2c_device.i2c.unlock()
            except Exception as e:
                self.get_logger().warn(f'Konnte I2C nicht korrekt freigeben: {e}')
            self.batterySensor = None

        if self.imu_publisher is not None:
            self.destroy_publisher(self.imu_publisher)
            self.imu_publisher = None

        self.lidar = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if rclpy.ok():
            self.get_logger().info(f"[{self.node_name}] on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('[SensorNode] Fehlerzustand!')
        return TransitionCallbackReturn.SUCCESS


    def __get_battery_pattern(self, percentage: int) -> int:
        try:
            for threshold, level in self.battery_levels:
                if percentage >= threshold:
                    return level
            return LEDPattern.BATTERY_0.value
        except Exception:
            self.get_logger().warn("Pattern nicht definiert bezogen auf Level {percentage}")

    def publish_battery_state(self):
        try:
            if self.batterySensor is not None:
                voltage = round(self.batterySensor.scaled_voltage(),2)
                current = round(self.batterySensor.read_current(),2)
                raw_v = round(self.batterySensor.read_voltage(),2)
                level = self.batterySensor.battery_level(voltage)       # die aktuelle Messung wird genutzt
                self.get_logger().info(f'(1) Battery State => ({level}%) | {voltage:.2f}V | {current:.2f}A')

                batStatus = BatteryStatus(voltage=voltage, level=level)
                pwrStatus = PowerStatus(current)

                batMsg = Battery()
                batMsg.battery_voltage = batStatus.voltage
                batMsg.battery_level = batStatus.level
                batMsg.battery_current = pwrStatus.current

                self.get_logger().info(f'Battery State => ({batMsg.battery_level}%) | {batMsg.battery_voltage:.2f}V | {batMsg.battery_current:.2f}A')
            else:
                self.get_logger().warn('BatterySensor nicht verfügbar ')
                batMsg = Battery()
                batMsg.battery_current = 0.0 # aktuell nicht genutzt
                batMsg.battery_voltage = 0.0
                batMsg.battery_level = 100

            level = random.randrange(0,100,10)
            ledMsg = LEDMessage()
            ledMsg.ledtype = 'WS2812'
            #
            # Battery-Level auf ein LEDPattern mappen (z.B 90% => LEDPattern.BATTERY_90 mit value 62)
            ledMsg.pattern = self.__get_battery_pattern(level)

            # ledMsg.brightness = 0.3
            # ledMsg.ledmask = 0b0000001000000100000010000001
            # ledMsg.timeout = 2000 + ((100 - level)*10)       # je schwächer die Batterie desto länger der Timeout
            # ledMsg.duration_on = 110 
            # ledMsg.duration_off = 110 - (100 - level)       # je schwächer die Batterie desto schneller blinkt es
            # ledMsg.pattern = LEDPattern.BATTERY_STATE.value

            self.get_logger().info(f"Level '{level} => Pattern : {ledMsg.pattern}'")
            self.led_publisher.publish(ledMsg)
            self.get_logger().info(f"Published LEDMessage() '{ledMsg}'")


        except Exception as e:
            self.get_logger().error(f'Fehler beim Lesen des Batteriesensors: {e}')
        
        # Publish direkt in topic
        try:
            self.battery_publisher.publish(batMsg)
        except Exception as e:
            self.get_logger().warn(f'Konnte Batteriestatus nicht veröffentlichen: {e}')


    def publish_imu_state(self):
        self.get_logger().debug('[SensorNode] read_and_publish_imu() – noch nicht implementiert')
        # try:
        #     self.imu_publisher.publish(msg)
        # except Exception as e:
        #     self.get_logger().warn(f'Konnte IMU-Daten nicht veröffentlichen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

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
