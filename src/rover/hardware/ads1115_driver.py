from .i2c_driver import SingletonI2CBus
import time
from enum import Enum

"""
Angepasste ADS1115Driver Klasse, da unter PI5 die adafruit Klasse nicht verwendet werden kann.
Verwendet wird smbus2, das kann die adafruit-klasse nicht.

Diese Klasse stellt drei Methoden zur Verfügung
scaled_voltage: skalierter Wertbereich in einer Range von 0-25V.
read_voltage: der tatsächliche berechnte Raw-Wert (in Volt) der am ADC anliegt
read_channel: liefert inen integer Wert zwische -32767 - +32768 zurück

"""

class ADS1115Driver:
    POINTER_CONVERT = 0x00
    POINTER_CONFIG = 0x01
    # Gain = +/- 4.096V (bitmask: 0b0100000000000000)
    # gain = +/- 6.144V
    #CONFIG_GAIN = 0x0000    # +- 6.144V
    CONFIG_DEFAULT = 0x8583  # Single-shot, AIN0, 128SPS, disable comparator
    #CONFIG_GAIN_FACTOR = 6.144

    ADS_GAINS = {
        0 : [0x0000, 6.144],  # 6.144V Messbereich
        1 : [0x0200, 4.096],  # 4.096V Messbereich
    }

    def __init__(self, logger, bus_id=1, slave_address=0x48, gain:int=0):
        self.logger = logger
        self.bus = SingletonI2CBus().getBus(bus_id)
        self.addr = slave_address
        self.logger.info("ADS1115Driver init")
        self.CONFIG_GAIN = (self.ADS_GAINS[gain][0] if gain in self.ADS_GAINS else self.ADS_GAINS[0][0])
        self.CONFIG_GAIN_FACTOR = (self.ADS_GAINS[gain][1] if gain in self.ADS_GAINS else self.ADS_GAINS[0][1])

    def read_voltage(self, channel : int = 0) -> float:
        """ Rückgabe des konvertierten Volt-Wertes zw. 0-5V basieren auf IN: 0-25.0V"""
        raw = self.read_channel(channel)
        return (raw * self.CONFIG_GAIN_FACTOR) / 32768.0

    def scaled_voltage(self, channel: int, voltMaxIn=25.0, voltMaxOut=5.0):
        """
        konvertiert den ADC-Wert aus read_voltage in einen Bereich zwischen MaxIn und MaxOut
        """
        sensor_voltage = self.read_voltage(channel=channel)
        return (sensor_voltage / voltMaxOut) * voltMaxIn

    def read_channel(self, channel: int) -> int:
        if not 0 <= channel <= 3:
            raise ValueError("Channel must be 0–3")

        mux = 0x4000 | (channel << 12)  # AINx vs GND
        config = (
            0x8000 |  # Start single conversion
            mux |
            self.CONFIG_GAIN |
            0x0100 |  # Mode = single-shot
            0x0080 |  # Data rate = 1600 SPS
            0x0003    # Disable comparator
        )

        config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
        self.bus.write_i2c_block_data(self.addr, self.POINTER_CONFIG, config_bytes)

        time.sleep(0.001)  # Wait at least 1ms

        result = self.bus.read_i2c_block_data(self.addr, self.POINTER_CONVERT, 2)
        value = (result[0] << 8) | result[1]

        if value > 0x7FFF:
            value -= 0x10000  # Convert to signed int

        return value

