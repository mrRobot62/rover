from enum import Enum
from ..hardware.ws2812_driver import WS2812


class LEDPattern(Enum):
    """
    Enumeration zur einfacheren Nutzung der unterschiedlichen Patterns
    """
    # Full color
    RED=0
    GREEN=1
    BLUE=2
    ORANGE=3
    WHITE=4

    # build-in pattern
    RAINBOW=5

    # Rover-Pattern
    BLINK_LEFT = 10
    BLINK_RIGHT = 11
    HAZARD_LIGHT = 12
    
    # Rover Info Pattern
    BATTERY_100 = 100
    BATTERY_90 = 101
    BATTERY_80 = 102
    BATTERY_70 = 103
    BATTERY_60 = 104
    BATTERY_50 = 105
    BATTERY_40 = 106
    BATTERY_30 = 106
    BATTERY_20 = 107
    BATTERY_10 = 108
    BATTERY_LOW = 109

    # Rover Error Codes
    DYNA_NOT_AVAILABLE = 200
    DYNA_PROBLEM = 201
    I2C_ESP32_NOT_FOUND = 250
    I2C_ESP32_STATE_ERR = 251

    PI_WIFI_NOT_AVAILABLE = 300

    UNKNOWN_ERROR = 1000
