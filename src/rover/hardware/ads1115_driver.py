import time
from enum import Enum

"""
Angepasste ADS1115Driver Klasse, da unter PI5 die adafruit Klasse nicht verwendet werden kann.
Verwendet wird smbus2, das kann die adafruit-klasse nicht.

Diese Klasse stellt Methoden zur Verfügung
scaled_voltage: skalierter Wertbereich in einer Range von 0-25V.
read_voltage: der tatsächliche berechnte Raw-Wert (in Volt) der am ADC anliegt
read_channel: liefert einen integer Wert zwische -32767 - +32768 zurück


ADS1115Driver wird vom I2C_Node.py instanziiert

"""
import time


import time
from smbus2 import SMBus

class ADS1115Driver:
    POINTER_CONVERT = 0x00
    POINTER_CONFIG = 0x01

    # Gain: [ConfigBits, maxVoltage]
    ADS_GAINS = {
        0: [0x0000, 6.144],  # ±6.144 V
        1: [0x0200, 4.096],  # ±4.096 V
    }

    MUX_MAP = {
        0: 0x4000,  # AIN0 vs GND
        1: 0x5000,  # AIN1 vs GND
        2: 0x6000,  # AIN2 vs GND
        3: 0x7000,  # AIN3 vs GND
    }

    def __init__(self, bus: SMBus, logger, slave_address=0x48, gain=0):
        self.logger = logger
        self.bus = bus
        self.addr = slave_address
        self.CONFIG_GAIN = self.ADS_GAINS.get(gain, self.ADS_GAINS[0])[0]
        self.CONFIG_GAIN_FACTOR = self.ADS_GAINS.get(gain, self.ADS_GAINS[0])[1]
        self.logger.info("ADS1115Driver initialized")

    def read_channel(self, channel: int) -> int:
        if channel not in self.MUX_MAP:
            raise ValueError("Channel must be 0–3")

        # Build config register
        config = (
            0x8000 |                        # OS = 1 (start single conversion)
            self.MUX_MAP[channel] |         # MUX config
            self.CONFIG_GAIN |              # PGA setting
            0x0100 |                        # MODE = 1 (single-shot)
            0x0080 |                        # Data rate = 1600 SPS
            0x0003                          # Disable comparator
        )
        config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
        self.bus.write_i2c_block_data(self.addr, self.POINTER_CONFIG, config_bytes)

        # Wait until conversion is ready (OS-Bit == 1)
        for _ in range(20):  # max 20ms timeout
            time.sleep(0.001)
            status = self.bus.read_i2c_block_data(self.addr, self.POINTER_CONFIG, 2)
            if status[0] & 0x80:  # OS-Bit == 1?
                break
        else:
            self.logger.warning(f"ADS1115: Timeout waiting for channel {channel} conversion")

        # Read result
        result = self.bus.read_i2c_block_data(self.addr, self.POINTER_CONVERT, 2)
        value = (result[0] << 8) | result[1]
        return value - 0x10000 if value > 0x7FFF else value

    def read_voltage(self, channel: int = 0) -> float:
        raw = self.read_channel(channel)
        voltage = (raw / 32768.0) * self.CONFIG_GAIN_FACTOR
        self.logger.debug(f"ADS1115: Channel {channel} raw={raw}, voltage={voltage:.4f} V")
        return voltage

    def scaled_voltage(self, channel: int, voltMaxIn=25.0, voltMaxOut=5.0) -> float:
        """
        Rechnet die gemessene Spannung hoch – z. B. bei Spannungsteiler.
        voltMaxIn = realer Messbereich (z. B. 25 V)
        voltMaxOut = Spannung am ADC bei max. Eingang (z. B. 5 V)
        """
        v = self.read_voltage(channel)
        factor = voltMaxIn / voltMaxOut
        scaled = v * factor
        self.logger.info(f"ADS1115: scaled_voltage channel {channel}: {scaled:.2f} V (raw={v:.3f} V)")
        return scaled




# class ADS1115Driver:
#     POINTER_CONVERT = 0x00
#     POINTER_CONFIG = 0x01

#     ADS_GAINS = {
#         0 : [0x0000, 6.144],  # ±6.144 V
#         1 : [0x0200, 4.096],  # ±4.096 V
#     }

#     def __init__(self, bus, logger, slave_address=0x48, gain=0):
#         self.logger = logger
#         self.bus = bus
#         self.addr = slave_address
#         self.logger.info("ADS1115Driver init")
#         self.CONFIG_GAIN = self.ADS_GAINS.get(gain, self.ADS_GAINS[0])[0]
#         self.CONFIG_GAIN_FACTOR = self.ADS_GAINS.get(gain, self.ADS_GAINS[0])[1]

#     def read_voltage(self, channel: int = 0) -> float:
#         raw = self.read_channel(channel)
#         return (raw * self.CONFIG_GAIN_FACTOR) / 32768.0

#     def scaled_voltage(self, channel: int, voltMaxIn=25.0, voltMaxOut=5.0):
#         sensor_voltage = self.read_voltage(channel=channel) #* voltMaxOut
#         self.logger.info(f"ADS1115Driver scaled_voltage: {sensor_voltage}V")
#         return sensor_voltage

#     def read_channel(self, channel: int) -> int:
#         if not 0 <= channel <= 3:
#             raise ValueError("Channel must be 0–3")

#         mux = 0x4000 | (channel << 12)
#         config = (
#             0x8000 | mux | self.CONFIG_GAIN |
#             0x0100 | 0x0080 | 0x0003
#         )
#         config_bytes = [(config >> 8) & 0xFF, config & 0xFF]

#         self.bus.write_i2c_block_data(self.addr, self.POINTER_CONFIG, config_bytes)
#         time.sleep(0.001)

#         result = self.bus.read_i2c_block_data(self.addr, self.POINTER_CONVERT, 2)
#         value = (result[0] << 8) | result[1]
#         return value - 0x10000 if value > 0x7FFF else value
