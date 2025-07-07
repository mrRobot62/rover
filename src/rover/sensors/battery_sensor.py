from smbus2 import SMBus
import board, busio
from ..hardware.ads1115_driver import ADS1115Driver
import time

import random

class BatterySensor:
    def __init__(self, 
                 logger, 
                 i2c_bus_id, 
                 batMin, 
                 batMax,
                 batVCh=1,
                 batCCh=2,
                 i2c_slave_address=0x48, 
                 gain=0b000):
        self.logger = logger
        self.batVCh = batVCh
        self.batCCh = batCCh
        self.batteryMin = (batMin if batMin > 0.0 else 0.1)
        self.batteryMax = (batMax if batMax > 0.0 else 0.1)
        self.ads_gain = gain

        self.ads = ADS1115Driver(
            logger=self.logger,
            bus_id=i2c_bus_id, 
            slave_address=i2c_slave_address, 
            gain=self.ads_gain)

    def voltage(self, channel=0) -> float:
        """ liese über Channel 0, die Spannung am Analogport A0 aus
        Davon ausgehend, das an A0 ein Voltage-Sensor angeschlossen ist.
        """
        v = self.ads.voltage(self.batVCh)
        #v = float(self.ads.read_channel(channel))
        return v

    def current(self, zero_offset=2.5, sensitivity=0.185, samples=10, delay=0.01) -> float:
        """ liest über Channel 1, die Stromstärke am Analogport A1 aus.
        Davon ausgehend das an A1 ein ACS712 angeschlossen ist.

        @param channel ID des Channels der ausgelesen werden soll 0-3
        @zero_offset Default 2.5 ggf. Kalibrieren, wenn bei 0A eine andere Spannung ausgegeben wird
        @sensitivity Default 0.185 bei ACS712-5A = 185mV/A, ACS712-30A = 100mV/A, ACS712-30A = 66mV/A
        """
        voltages=[]
        for _ in range(samples):
            voltages.append(self.voltage(self.batCCh))
            time.sleep(delay)

        avg_voltage = sum(voltages) / len(voltages)
        current = (avg_voltage - zero_offset) / sensitivity
        return round(current, 3)
        
    def battery_level(self) -> int:

        # Begrenzen auf den Spannungsbereich
        clamped = max(min(self.voltage(), self.batteryMax), self.batteryMin)

        # Prozentsatz berechnen
        level = ((clamped - self.batteryMin) / (self.batteryMax - self.batteryMin)) * 100.0

        return int(round(level))