from enum import Enum
import struct
from typing import List

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

# Unterstützte Pins (für Prüfung und Symbolik)
class ESP32PINS(Enum):
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

class SERVICE_RESPONSE(Enum):
    ROS_SERVICE_NOT_AVAILABEL = -100
    SLAVE_RESPONSE_EMPTY = -101
    I2C_ERROR=-200
    

class ESP32_RESPONSE(Enum):
    OK=0
    DYNA_ERROR=-10
    IO_ERROR=-20
    FW_ERROR=-30
    UNKNOWN_CMD=-40
    UNKNOWN_SCMD=-41