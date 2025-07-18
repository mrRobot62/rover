from rover_interfaces.msg import BatteryRaw
import rclpy
from typing import List
from rclpy.node import Node
from .led_pattern import LEDPattern


"""
SensorNode
 └── BatteryClient
      └── ruft subscribe BatteryRaw
            └── empfängt die Nachricht vom I2CNode bereitet die Daten auf 
                  und ruft eine Callback im SensorNode auf

"""

class BatteryClient:

    def __init__(self, 
                 logger,
                 node, 
                 topic: str, 
                 batMin: float, 
                 batMax: float,
                 callback=None,
                 ):
        self.logger = logger
        self.node = node
        self.callback = callback
        self.batteryMin = batMin
        self.batteryMax = batMax
        self.topic = topic
        self.subscription = self.node.create_subscription(
            BatteryRaw,
            topic,
            self._on_battery_voltage_msg,
            10)
        self.logger.info(f"BatteryClient() - init - '{self.callback}', '{self.topic}', '{self.batteryMin}, '{self.batteryMax}'")
                         

    def _on_battery_voltage_msg(self, msg: BatteryRaw):
        # channel_0 enthält schon den skalierte Voltage bereich
        # IN_MIN = 0.0V, IN_MAX: 25.0V
        # scaled_voltage skaliert nun den gemessen Betrag (irgendwas zw. 0.0 - 5.0V)
        # in den Messbereich von 0-25V
        voltage = round(msg.channel_0, 2)
        current = round(msg.channel_1, 3)
        #
        # prüfen ob voltage überhaupt im Bereich von batMin und batMax liegt, wenn nein, wird angepasst
        clamped = max(min(voltage, self.batteryMax), self.batteryMin)
        # Prozentsatz berechnen
        level = ((clamped - self.batteryMin) / (self.batteryMax - self.batteryMin)) * 100.0
        if self.callback:
            self.callback(voltage, level, current)
            #self.logger.info(f"BatteryClient - '{voltage}V', '{level}%', '{current}A' => {self.callback}")
        else:
            self.logger.info(f"BatteryClient - no callback '{self.callback}'")
    
