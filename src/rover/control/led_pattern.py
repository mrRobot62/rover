from enum import Enum

class LEDPattern(Enum):
    """
    Enumeration zur einfacheren Nutzung der unterschiedlichen Patterns
    """
    NONE=0
    OFF=1
    UNKNOWN_ERROR=10


    # fill()
    RED=50
    GREEN=51
    BLUE=52
    YELLOW=53
    PINK=54
    WHITE=55
    AQUA=56
    LILA=57
    GREENYELLOW=58
    GREENBLUE=59
    ORANGE=60

    # Battery-Level
    BATTERY_100=61
    BATTERY_90=62
    BATTERY_80=63
    BATTERY_70=64
    BATTERY_60=65
    BATTERY_50=66
    BATTERY_40=67
    BATTERY_30=68
    BATTERY_20=105
    BATTERY_10=104

    # blink()
    HAZARD = 100
    BLINK_LEFT = 101
    BLINK_RIGHT = 102
    
    # blink()
    BATTERY_0 = 103






