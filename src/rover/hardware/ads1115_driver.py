from .i2c_driver import SingletonI2CBus
import time

class ADS1115Driver:
    POINTER_CONVERT = 0x00
    POINTER_CONFIG = 0x01
    # Gain = +/- 4.096V (bitmask: 0b0100000000000000)
    # gain = +/- 6.144V
    CONFIG_GAIN = 0x0000    # +- 6.144V
    CONFIG_DEFAULT = 0x8583  # Single-shot, AIN0, 128SPS, disable comparator
    CONFIG_GAIN_FACTOR = 6.144

    def __init__(self, logger, bus_id=1, slave_address=0x48, gain:int=0):
        self.logger = logger
        self.bus = SingletonI2CBus().getBus(bus_id)
        self.addr = slave_address
        self.logger.info("ADS1115Driver init")


    def voltage(self, channel : int) -> float:
        """ Rückgabe des konvertierten Volt-Wertes"""
        raw = self.read_channel(channel)
        return raw * self.CONFIG_GAIN_FACTOR / 32768.0

    def current    (self, channel : int) -> float:
        """ Rückgabe des konvertierten Strom-Wertes"""
        raw = self.read_channel(channel)
        return raw * self.CONFIG_GAIN_FACTOR / 32768.0



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

