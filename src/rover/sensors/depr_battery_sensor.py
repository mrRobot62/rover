from smbus2 import SMBus
import board, busio
from ..hardware.ads1115_driver import ADS1115Driver
import time

import random

from dataclasses import dataclass

@dataclass
class BatteryStatus:
    voltage: float           # Spannung in V
    level: int               # Füllstand in Prozent (0-100)
    is_low: bool = False     # true, wenn Batterie niedrig
    is_critical: bool = False # true, wenn Batterie kritisch

@dataclass
class PowerStatus:
    current: float           # Strom in A
    is_high: bool = False    # True wenn > 4.0A

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
        self.batteryMin = (batMin if batMin > 0.0 else 0.1) # Division durch 0 vermeiden
        self.batteryMax = (batMax if batMax > 0.0 else 0.1) # Division durch 0 vermeiden
        self.ads_gain = gain
        self.acs712_vdd_key = int(acs712_vdd * 10)


        self.ads = ADS1115Driver(
            logger=self.logger,
            bus_id=i2c_bus_id, 
            slave_address=i2c_slave_address, 
            gain=self.ads_gain)

        # key = acs712_type, Value= Skalierungs-Faktor
        self.__acs712_scales = {
            5:0.185,
            20:0.100,
            30:0.66
        }

        # Key = acs712_vdd, value = zero_offset
        self.__acs712_offsets = {
            50 : 2.5,
            33 : 1.25
        }

        self.acs712_sensitivity = (self.__acs712_scales[acs712_type] if acs712_type in self.__acs712_scales else 0.185)
        self.acs712_zero_offset = (self.__acs712_offsets[self.acs712_vdd_key] if self.acs712_vdd_key in self.__acs712_offsets else 0.185)

    def read_voltage(self, channel=None) -> float:
        """ liest über channel x, die Spannung am Analogport Ax aus
            Davon ausgehend, das an Ax ein Voltage-Sensor angeschlossen ist.
            Wichtig zwischen zwei Aufrufen müssen mindestens 10ms vergehen, sonst bekomme man Fehlemessungen
        
        """
        channel = (self.batVCh if channel is None else channel)
        v = self.ads.read_voltage(channel)
        return v

    def scaled_voltage(self, channel=None):
        """ skaliert den Voltage-Wert in einen Bereich zwischen 0.0 und voltMaxIn basieren auf voltMaxOut
            Wichtig zwischen zwei Aufrufen müssen mindestens 10ms vergehen, sonst bekomme man Fehlemessungen
        
        """
        channel = (self.batVCh if channel is None else channel)
        v = self.ads.scaled_voltage(channel, voltMaxIn=self.voltMaxIn, voltMaxOut=self.voltMaxOut)
        self.logger.debug(f"scaled_voltage(), Ch:{channel}, V:{v}, vIn:{self.voltMaxIn}, vOut: {self.voltMaxOut}")
        return v

    def read_current(self, channel=None) -> float:
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
            voltages.append(self.read_voltage(channel))
            time.sleep(delay)

        avg_voltage = sum(voltages) / len(voltages)
        current = (avg_voltage - self.acs712_zero_offset) /  self.acs712_sensitivity
        return round(current, 3)
        
    def battery_level(self, channel=None) -> int:
        """ liest den ADS neu aus und brechnet dann den Level.
            Wichtig zwischen zwei Aufrufen müssen mindestens 10ms vergehen, sonst bekomme man Fehlemessungen
        """
        channel = (self.batVCh if channel is None else channel)
        # Begrenzen auf den Spannungsbereich
        scaled = self.scaled_voltage(channel)
        return self.battery_level(scaled)
    
    def battery_level(self, voltage: float) -> int:
        """ berechnet den Level basieren auf die übergeben Spannung"""
        scaled = voltage
        clamped = max(min(scaled, self.batteryMax), self.batteryMin)

        # Prozentsatz berechnen
        level = ((clamped - self.batteryMin) / (self.batteryMax - self.batteryMin)) * 100.0
        self.logger.debug(f"battery_level()Scaled: {scaled}, Clamped: {clamped}, level: {int(level)} Volt:{self.voltMaxOut}|{self.voltMaxIn}, Bat: {self.batteryMin}|{self.batteryMax}")
        return int(round(level))

    def get_battery_status(self, voltage_channel : int) -> BatteryStatus:
        """ 
        gibt ein BatteryStatus Objekt zurück. Liest von von voltage_channel
        @return BatteryStatus
        """
        v = round(self.scaled_voltage(voltage_channel),2)
        l = self.battery_level(v)
        isLow = (True if v <= self.batteryMin else False)
        isCritical = (True if v <= (self.batteryMin-0.2) else False)
        return BatteryStatus(
            voltage = max(v, 0.0),
            level=l,
            is_low=isLow,
            is_critical=isCritical
        )
    
    def get_power_consumption(self, current_channel : int, high_power: float = 4.0) -> PowerStatus:
        """ 
        
        """
        c=round(self.read_current(current_channel), 2)
        c=max(c,0.0)
        is_high=(True if c > high_power else False)
        return PowerStatus(
            current=c,
            is_high=is_high
        )