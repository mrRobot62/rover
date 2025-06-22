from smbus2 import SMBus
import board, busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import random

class BatterySensor:
    def __init__(self, chan=0, gain=1, i2c_bus=None):
        i2c = i2c_bus or busio.I2C(board.SCL, board.SDA)
        try:
            self.ads = ADS.ADS1115(i2c)
            self.ads.gain = gain
            channel = getattr(ADS, f"P(chan)")
            self.analog = AnalogIn(self.ads, channel)
        except Exception as err:
            print (f"NO ADS1115 found")

    def read_voltage(self):
        voltage = float(random.randrange(1360, 1610))
        voltage /= 100.0
        return voltage
        #return self.analog.voltage
    
