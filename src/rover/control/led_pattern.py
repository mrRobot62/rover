from enum import Enum

class LEDPattern(Enum):
    """
    Enumeration zur einfacheren Nutzung der unterschiedlichen Patterns
    """
    # fill()
    RED=0
    GREEN=1
    BLUE=2
    ORANGE=3
    WHITE=4

    # blink()
    BLINK_LEFT = 10
    BLINK_RIGHT = 11
    HAZARD_LIGHT = 12
    
    # fill()
    BATTERY_100 = 100
    BATTERY_90 = 101
    BATTERY_80 = 102
    BATTERY_70 = 103
    BATTERY_60 = 104
    BATTERY_50 = 105
    BATTERY_40 = 106
    BATTERY_30 = 106
    BATTERY_20 = 107
    
    # blink()
    BATTERY_10 = 108
    BATTERY_LOW = 109

    # Rover - driver_controller_node
    # blink()
    DYNA_NOT_AVAILABLE = 200
    DYNA_PROBLEM = 201
    I2C_ESP32_NOT_FOUND = 250
    I2C_ESP32_STATE_ERR = 251

    # Rover - navigation_node
    # 300-399
    #     
    # Rover - vision_node
    # 400 - 499

    # Rover - odom_node
    # 500 - 599

    # Rover
    # Software
    PI_WIFI_NOT_AVAILABLE = 500

    # circle()
    ROVER_BOOT1 = 510
    ROVER_BOOT2 = 511
    ROVER_BOOT3 = 512
    ROVER_SHUTDOWN1 = 515
    ROVER_SHUTDOWN2 = 516


    UNKNOWN_ERROR = 1000
    STOP = 9999


