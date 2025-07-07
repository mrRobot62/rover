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
                 voltMaxIn=25.0,
                 voltMaxOut=5.0,
                 batVCh=1,
                 batCCh=2,
                 acs712_type="5A",
                 acs712_vdd=5.0,
                 acs712_vout=5.0,
                 i2c_slave_address=0x48, 
                 gain=0):
        self.logger = logger
        self.batVCh = batVCh
        self.batCCh = batCCh
        self.voltMaxIn = voltMaxIn
        self.voltMaxOut = voltMaxOut
        self.batteryMin = (batMin if batMin > 0.0 else 0.1)
        self.batteryMax = (batMax if batMax > 0.0 else 0.1)
        self.ads_gain = gain
        self.acs712_vdd_key = int(acs712_vdd * 10)


        self.ads = ADS1115Driver(
            logger=self.logger,
            bus_id=i2c_bus_id, 
            slave_address=i2c_slave_address, 
            gain=self.ads_gain)

        # key = acs712_type, Value= Skalierungs-Faktor
        self.__acs712_scales = {
            "5A":0.185,
            "20A":0.100,
            "30A":0.66
        }

        # Key = acs712_vdd, value = zero_offset
        self.__acs712_offsets = {
            50 : 2.5,
            33 : 1.25
        }

        self.acs712_sensitivity = (self.__acs712_scales[acs712_type] if acs712_type in self.__acs712_scales else 0.185)
        self.acs712_zero_offset = (self.__acs712_offsets[self.acs712_vdd_key] if self.acs712_vdd_key in self.__acs712_offsets else 0.185)

    def voltage(self, channel=None) -> float:
        """ liest über channel x, die Spannung am Analogport Ax aus
        Davon ausgehend, das an Ax ein Voltage-Sensor angeschlossen ist.
        """
        channel = (self.batVCh if channel is None else channel)
        v = self.ads.read_voltage(channel)
        return round(v, 2)

    def scaled_voltage(self, channel=None):
        """ skaliert den Voltage-Wert in einen Bereich zwischen 0.0 und voltMaxIn basieren auf voltMaxOut"""
        channel = (self.batVCh if channel is None else channel)
        v = self.ads.scaled_voltage(channel, voltMaxIn=self.voltMaxIn, voltMaxOut=self.voltMaxOut)
        return round(v,2)

    def current(self, channel=None) -> float:
        """ liest über Channel 1, die Stromstärke am Analogport A1 aus.
        Davon ausgehend das an A1 ein ACS712 angeschlossen ist.

        @param channel ID des Channels der ausgelesen werden soll 0-3
        @zero_offset Default 2.5 ggf. Kalibrieren, wenn bei 0A eine andere Spannung ausgegeben wird
        @sensitivity Default 0.185 bei ACS712-5A = 185mV/A, ACS712-30A = 100mV/A, ACS712-30A = 66mV/A
        """
        channel = (self.batCCh if channel is None else channel)
        samples=10
        delay = 0.01
        voltages=[]
        for _ in range(samples):
            voltages.append(self.voltage(channel))
            time.sleep(delay)

        avg_voltage = sum(voltages) / len(voltages)
        current = (avg_voltage - self.acs712_zero_offset) /  self.acs712_sensitivity
        return round(current, 3)
        
    def battery_level(self) -> int:

        # Begrenzen auf den Spannungsbereich
        clamped = max(min(self.voltage(), self.batteryMax), self.batteryMin)

        # Prozentsatz berechnen
        level = ((clamped - self.batteryMin) / (self.batteryMax - self.batteryMin)) * 100.0

        return int(round(level))