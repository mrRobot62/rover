"""
i2c_driver.py

Dieses Modul stellt grundlegende I2C-Kommunikationsfunktionen bereit, um Datenpakete 
zwischen einem Raspberry Pi (Host) und einem ESP32-Mikrocontroller auszutauschen. 
Die Kommunikation erfolgt über ein eigenes binäres Protokoll mit Header, Payload und CRC8.

Unterstützte Funktionen:
- Senden von Kommandos (z. B. Servo-Werte, digitale/analoge IO)
- Prüfen auf valide Pins
- Umrechnung von Gleitkommazahlen für binäre Übertragung
- Erkennung und Initialisierung des I2C-Busses
- Abstraktion für zukünftige Treiber-Klassen (z. B. Servosteuerung)

Geeignet für embedded Steuerung von Robotik-Systemen mit Echtzeitanforderungen.
"""

from abc import ABC, abstractmethod
from smbus2 import SMBus, i2c_msg
import smbus2
import json
import time
import launch.logging
import subprocess
import os
import sys
from enum import Enum
import struct
from typing import List

# Konstanten zur Bus- und Adresskonfiguration
I2C_BUS = "/dev/i2c-"      # Pfad zum I2C-Device auf Linux
I2C_BUS_ID = 1             # I2C-Busnummer (i.d.R. 1 für RPi)
I2C_ADDRESS = 0x12         # Zieladresse (ESP32-Slave)

# Kommando-IDs für die I2C-Kommunikation
class CommandID(Enum):
    NONE=0
    SERVO_WRITE=1
    SERVO_READ=2
    DIGITAL_WRITE=3
    DIGITAL_READ=4
    ANALOG_WRITE=5
    ANALOG_READ=6
    ESP32_STATE=7

# Subkommandos für erweiterte Steuerung
class SubCommandID(Enum):
    SCMD_NONE=0
    SCMD_SERVO_SPEED=1
    SCMD_SERVO_POSITION=2
    SCMD_SERVO_SPEED_POSITION=3
    SCMD_SERVO_TORQUE_ENABLE=4
    SCMD_SERVO_LED_ON=5

# Unterstützte Digitalpins (für Prüfung und Symbolik)
class DIGITALPINS(Enum):
    NONE=0
    ONBOARD_LED=2
    LED1=19
    LED2=18
    IO1=5
    IO2=4
    IO3=15
    IO4=23
    ADC0=36
    ADC1=39
    ADC2=34
    ADC3=35

# Merkt sich das zuletzt gesendete Paket zur Vermeidung von Duplikaten
_last_packet = {
    "cmd": None,
    "subcmd": None,
    "data": None
}

def getMsg(txt, prefix='[?]', suffix='', line_break=True):
        """
        erzeugt eine log textzeile und gibt diese zurück
        """
        if line_break:
            line_break = '\n'
        else:
            line_break = ''
            txt = f"{line_break}{prefix} {txt} {suffix}"
        return txt
def isValidPinState(pins: List[int], states: List[int]):
    """
    Prüft, ob die angegebenen Pins gültig sind. Falls nicht, werden Pin und zugehöriger Zustand auf 0 gesetzt.
    Rückgabe: bereinigte Listen von Pins und Zuständen.
    """
    if pins[0] not in (item.value for item in DIGITALPINS):
        pins[0] = 0
        states[0] = 0
    if pins[1] not in (item.value for item in DIGITALPINS):
        pins[1] = 0
        states[1] = 0
    return pins, states

def dataModulo(data_vals, factor=100):
    """
    Skaliert Float-Werte in einen Bereich von uint16_t (0–65535), unter Verwendung von Modulo-Arithmetik.
    Wird z. B. benötigt, um negative Geschwindigkeits- oder Lenkwinkelwerte als unsigned zu übertragen.

    @param data_vals ist ein Array von Float-Werten
    @param factor default 100 = Multiplikator
    @return array der neu berechneten Werte
    """
    return [int(val * factor) % 65536 for val in data_vals]

def send_packet(bus:smbus2.SMBus, data_vals, cmd, subcmd=0, flags=0):
    """
    Erzeugt ein binäres Paket mit Header, Kommando, Daten und CRC8 und sendet es über den I2C-Bus.
    Wiederholte Pakete mit identischem Inhalt werden nicht erneut gesendet.
    """
    global _last_packet

    if data_vals is None:
        # Falls None übergeben wird leer initialisieren
        data_vals = []

    # zur Sicherheit prüfen ob auch tatsächlich 5 Werte im Array enthalten sind
    # data_vals[:5]                   schneidet alles > 5 ab
    # [0] * (5 - len(data_vals))      fügt soviele 0 hinzu bis 5 erreicht sind
    # if data_vals else [0] * 5       ersetzt ein leeres Array durch fünf Nullen
    data_vals = data_vals[:5] + [0] * (5 - len(data_vals)) if data_vals else [0] * 5

    # Duplikatsprüfung
    # vermeidet das das gleiche Command mehrfach gesendet wird
    if (_last_packet["cmd"] == cmd and
        _last_packet["subcmd"] == subcmd and
        _last_packet["data"] == data_vals):
        return 0

    _last_packet["cmd"] = cmd
    _last_packet["subcmd"] = subcmd
    _last_packet["data"] = list(data_vals)  # WICHTIG: echte Kopie nötig

    header = 0xFEEF
    reserved = 0x00

    # Struktur des Pakets: <HBBB5HB -> Header, cmd, subcmd, flags, 5x uint16_t, reserved
    # Insgesamt 16Bytes + CRC(Byte)
    payload = struct.pack(
        "<HBBB5HB",       # Format-String:
                        # <: Little Endian (LSB first)
                        # H: 16-bit unsigned short für den Header (z. B. 0xFEEF)
                        # B: 8-bit unsigned für cmd (CommandID)
                        # B: 8-bit unsigned für subcmd (SubCommandID)
                        # B: 8-bit unsigned für flags (z. B. 0x00)
                        # 5H: fünf 16-bit unsigned shorts für die Nutzdaten
                        # B: 8-bit unsigned für reserved (z. B. 0x00)
        header,           # 0xFEEF, zur Synchronisation auf ESP32-Seite
        cmd,              # Hauptkommando, z. B. CommandID.SERVO_WRITE.value
        subcmd,           # Subkommando, z. B. SubCommandID.SCMD_SERVO_SPEED.value
        flags,            # Steuerflags, meist 0x00
        *data_vals,       # Entpackt Liste von 5 Werten (uint16_t), z. B. [100, 0, 0, 0, 0]
        reserved          # Reserviertes Byte (z. B. für zukünftige Erweiterung)
    )    
    crc = CRC8(payload)
    packet = payload + bytes([crc])

    print(f"→ Sende Paket: {[hex(b) for b in packet]}")

    try:
        msg = i2c_msg.write(I2C_ADDRESS, packet)
        bus.i2c_rdwr(msg)
        return 0
    except Exception as e:
        print(f"I2C-Error => {e}")
        return 1

def CRC8(data: bytes) -> int:
    """
    Berechnet eine CRC8-Prüfsumme über das gegebene Daten-Bytearray.
    Polynom: 0x31
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

class I2CTest():
    """
    Hilfsklasse zur Diagnose und Initialisierung des I2C-Busses (Laden von Treibern, Geräteprüfung, etc.).
    """
    def __init__(self, i2c_bus=I2C_BUS):
        self.bus = smbus2.SMBus(i2c_bus)
        self.logger = launch.logging.get_logger('I2CTest')

    def check_i2c_dev_module(self):
        """
        Prüft, ob das Kernelmodul 'i2c_dev' geladen ist. Lädt es ggf. nach.
        """
        self.logger("[I2CTest] => Prüfe Kernel-Modul 'i2c_dev'...")
        result = subprocess.run(["lsmod"], capture_output=True, text=True)
        if "i2c_dev" in result.stdout:
            self.logger("✓ i2c_dev Modul geladen")
        else:
            self.logger("❌ i2c_dev Modul nicht geladen")
            subprocess.run(["sudo", "modprobe", "i2c_dev"])
            self.logger("✓ Modul geladen")
            return -1
        return 0

    def check_i2c_device(self):
        """
        Prüft, ob das I2C-Gerät im Dateisystem existiert.
        """
        self.logger(f"[I2CTest] => Prüfe I2C Device {I2C_BUS}...")
        if os.path.exists(I2C_BUS):
            self.logger(f"✓ {I2C_BUS} vorhanden")
        else:
            self.logger(f"❌ {I2C_BUS} nicht vorhanden")
            return -2
        return 0

    def check_all_i2c_busses(self):
        """
        Gibt alle verfügbaren I2C-Geräte im System aus.
        """
        self.logger("[I2CTest] => Gefundene I2C Busse:")
        os.system("ls /dev/i2c-*")
        return 0

    def run_i2cdetect(self):
        """
        Führt das Kommandozeilen-Tool `i2cdetect` aus und prüft, ob der ESP32 erreichbar ist.
        """
        self.logger(f"[I2CTest] => Starte i2cdetect auf {I2C_BUS}...")
        result = subprocess.run(["i2cdetect", "-y", "1"], capture_output=True, text=True)
        self.logger(result.stdout)
        if I2C_ADDRESS in result.stdout:
            self.logger(f"✓ ESP32 gefunden bei Adresse {I2C_ADDRESS}")
        else:
            self.logger(f"❌ Kein Gerät bei Adresse {I2C_ADDRESS} gefunden")
            return -3
        return 0

class SingletonI2CBus:
    """
    Singleton-Wrapper für den I2C-Bus. Stellt sicher, dass nur eine Bus-Instanz existiert.
    """
    _bus_instance = None

    @classmethod
    def getBus(cls, i2c_bus_id=I2C_BUS_ID, i2c_address=I2C_ADDRESS):
        if cls._bus_instance is None:
            cls._bus_instance = smbus2.SMBus(i2c_bus_id)
        return cls._bus_instance



class ESP32RawDriver(ABC):
    """
    Abstrakte Basisklasse zur Kommunikation mit dem ESP32.
    Bietet Grundfunktionen für digitale und analoge IO-Kommandos.
    """
    def __init__(self, logger, i2c_bus_id=I2C_BUS_ID, i2c_address=I2C_ADDRESS):
        self.bus = SingletonI2CBus.getBus(i2c_bus_id)
        self.address = i2c_address
        self.logger = logger
        self.logger.info(getMsg(f"I2CBus ({self.bus}): ' {I2C_BUS}{self.bus.fd}. Slave: {self.address}", '[ESP32RawDriver]'))

    def digitalWrite(self, pins: List[int] = [0, 0], states: List[int] = [0, 0]):
        """
        Setzt zwei digitale Ausgänge (HIGH/LOW) auf dem ESP32.
        """
        cmd = CommandID.DIGITAL_WRITE.value
        scmd = SubCommandID.SCMD_NONE.value
        pins, states = isValidPinState(pins, states)
        states = [x if x in (0, 1) else 0 for x in states]
        data = pins + states
        print(f"=> ({data})")
        return send_packet(self.bus, data_vals=data, cmd=cmd, subcmd=scmd, flags=0x00)

    def digitalRead(self, pins: List[int] = [0, 0]):
        """
        Liest zwei digitale Eingänge (HIGH/LOW) vom ESP32.
        (Platzhalter: keine echte Rückgabe implementiert)
        """
        state = [0, 0]
        cmd = CommandID.DIGITAL_READ
        scmd = SubCommandID.SCMD_NONE
        pins, states = isValidPinState(pins, state)
        return state  # TODO: read-back implementieren

    def analogWrite(self, pins: List[int] = [0, 0], state: List[int] = [0, 0]):
        """
        Setzt zwei analoge Ausgänge (PWM) mit Werten zwischen 0–4095.
        """
        cmd = CommandID.ANALOG_WRITE
        scmd = SubCommandID.SCMD_NONE
        pins = [p if 0 <= p <= 50 else 0 for p in pins]
        state = [s if 0 <= s <= 4095 else 0 for s in state]
        data = pins + state
        return send_packet(self.bus, data_vals=data, cmd=cmd, subcmd=int(scmd), flags=0x00)

    def analogRead(self, pins: List[int] = [0, 0]):
        """
        Liest zwei analoge Eingänge vom ESP32. (Platzhalterfunktion)
        """
        state = [0, 0]
        cmd = CommandID.ANALOG_READ
        scmd = SubCommandID.SCMD_NONE
        pins = [p if 0 <= p <= 50 else 0 for p in pins]
        return state  # TODO: read-back implementieren

class ServoDriver():
    """
    Konkreter Treiber zur Steuerung von Dynamixel-Servos über den ESP32.
    Nutzt binäre I2C-Kommandos zur Übergabe von Geschwindigkeit, Position, etc.
    """
    def __init__(self, logger, i2c_bus_id=I2C_BUS_ID, i2c_address=I2C_ADDRESS):
        self.bus = SingletonI2CBus.getBus(i2c_bus_id)
        self.address = i2c_address
        self.logger = logger
        self.logger.info(getMsg(f"I2CBus ({self.bus}): ' {I2C_BUS}{self.bus.fd}. Slave: {self.address}",'[ServoDriver]' ))

    def write(self, cmd: CommandID, scmd: SubCommandID, servo_data: dict, force=False):
        """
        Wandelt ein Servo-Datenpaket (z. B. Geschwindigkeit, Lenkwinkel) in ein binäres Paket
        um und sendet es an den ESP32. Erwartet ein Dictionary mit Floatwerten im Bereich [-1.0, 1.0].
        """
        if cmd == CommandID.SERVO_WRITE:
            data = dataModulo(data_vals=servo_data, factor=100)
            rc = send_packet(self.bus, data_vals=data, cmd=cmd.value, subcmd=scmd.value, flags=0x00)
            self.logger.debug(getMsg(f"\t{cmd}:{scmd} Data:{servo_data}", '[ServoDriver]'))
            return rc

        self.logger.error(getMsg(f"::write aber cmd={cmd} - Fehler", '[ServoDriver]'))
        return 1
