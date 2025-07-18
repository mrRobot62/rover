import threading
import time
from rover_interfaces.msg import BatteryRaw, ACS712
from .utilities import Utilities
"""

WICHTIG:
der ACS712 ist im Rover-Projekt über einen ADS1115 angeschlossen (Channel A1).
Daher bietet die Klasse auch keinerlei direkten Zugriffe auf einen ADC.

Diese Klasse wird vom ADC1115 geladen und genutzt
"""

class ACS712Client:


    def __init__(self, 
                 node, 
                 logger, 
                 publish_topic: str, 
                 subscribe_topic:str, 
                 channel:int=1, 
                 zero_offset: float = 2.5,
                 acs712_type:int=5,
                 callback=None
                 ):
        self.node = node
        self.logger = logger
        self.subscribe_topic = subscribe_topic
        self.publish_topic = publish_topic
        self.callback = callback
        self.channel = channel
        self.zero_offset = zero_offset
        self.acs712_type = acs712_type
        self.acs712_sensitivy = Utilities.get_value_or_default(
            my_dict=self.acs712_type_sensitivity,
            default_key=5
        )
        
        self.subscription = self.node.create_subscription(
            BatteryRaw,
            self.subscribe_topic,
            self._on_battery_current_raw_msg,
            10
        )



    def get_channel_value(self, msg, channel_id):
        attr_name = f"channel_{channel_id}"
        return getattr(msg, attr_name, None)  # None ist der Default, falls Attribut nicht existiert

    def _on_battery_current_raw_msg(self, msg: BatteryRaw):
        """ liest Daten die der ADS1115 über den I2CNode zur Verfügung stellt. Der ACS712 ist an Channel 1 installiert """
        voltage = self.get_channel_value(msg, self.channel)
        

