from enum import Enum

class LEDPattern(Enum):
    """
    Enumeration zur einfacheren Nutzung der unterschiedlichen Patterns
    """
    # DEMO
    RED=0
    GREEN=1
    BLUE=2
    ORANGE=3
    WHITE=4
    RAINBOW=4

    # Blinkende Patterns
    BLINK_LEFT = 10
    BLINK_RIGHT = 11
    HAZARD_LIGHT = 12
    
    # Info Pattern
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

    # Error Codes
    DYNA_NOT_AVAILABLE = 200
    DYNA_PROBLEM = 201
    I2C_ESP32_NOT_FOUND = 250
    I2C_ESP32_STATE_ERR = 251

    PI_WIFI_NOT_AVAILABLE = 300

    UNKNOWN_ERROR = 1000

# -------------------------------------------------------------------------------------------------------------------------------
class LEDPatternConfig:
    def __init__(self, pattern_id: int, pattern_name=str, led_type="WS2812", duration: int=0, timeout: int=0):
        """ 
        LEDPattern.
        @param pattern_id entspricht der ID aus der Enumeration
        @param pattern_name Name des Musters
        @param duration default=0 (für das Pattern irrelevant), wenn > 0 wird diese Zeit in ms als Pause genutzt für das nachfolgende Pattern (zB BLINK_xxx)#
        @param timeout default=0 (für das Pattern irrelevant), wenn > 0 die Zeit in ms, bis das Pattern deaktiviert (LED OFF) gesetzt wird
        """
        self.pattern_id = pattern_id
        self.pattern_name = pattern_name
        self.led_type = led_type
        self.duration = duration
        self.timeout = timeout

# -------------------------------------------------------------------------------------------------------------------------------
LED_PATTERNS_DICT = {
    LEDPattern.RED: LEDPatternConfig(LEDPattern.RED, "DEMO RED", duration=0, timeout=2000),
    LEDPattern.GREEN: LEDPatternConfig(LEDPattern.GREEN, "DEMO GREEN", duration=0, timeout=2000),
    LEDPattern.BLUE: LEDPatternConfig(LEDPattern.BLUE, "DEMO BLUE", duration=0, timeout=2000),
    LEDPattern.ORANGE: LEDPatternConfig(LEDPattern.ORANGE, "DEMO ORANGE", duration=0, timeout=2000),
    LEDPattern.WHITE: LEDPatternConfig(LEDPattern.WHITE, "DEMO WHITE", duration=0, timeout=2000),
    LEDPattern.RAINBOW: LEDPatternConfig(LEDPattern.RAINBOW, "DEMO RAINBOW", duration=0, timeout=3000),

    LEDPattern.BLINK_LEFT: LEDPatternConfig(LEDPattern.BLINK_LEFT, "BLINKER LINKS", duration=250, timeout=0),
    LEDPattern.BLINK_RIGHT: LEDPatternConfig(LEDPattern.BLINK_RIGHT, "BLINKER RECHTS", duration=250, timeout=0),
    LEDPattern.HAZARD_LIGHT: LEDPatternConfig(LEDPattern.HAZARD_LIGHT, "WARNBLINKER", duration=250, timeout=0),
    LEDPattern.BATTERY_100: LEDPatternConfig(LEDPattern.BATTERY_100, "Batterie 100%", duration=0, timeout=1000),
    LEDPattern.BATTERY_100: LEDPatternConfig(LEDPattern.BATTERY_50, "Batterie 50%", duration=0, timeout=1000),
    LEDPattern.BATTERY_100: LEDPatternConfig(LEDPattern.BATTERY_20, "Batterie 20%", duration=250, timeout=3000),
    LEDPattern.BATTERY_100: LEDPatternConfig(LEDPattern.BATTERY_10, "Batterie 10%", duration=250, timeout=5000),
    LEDPattern.BATTERY_LOW: LEDPatternConfig(LEDPattern.BATTERY_LOW, "Batterie LOW", duration=250, timeout=0),
    
    LEDPattern.DYNA_NOT_AVAILABLE: LEDPatternConfig(LEDPattern.DYNA_NOT_AVAILABLE, "Dyna-Servos nicht gefunden", duration=0, timeout=0)

}
    
def get_led_pattern(id : LEDPattern) -> LEDPatternConfig:
    return LED_PATTERNS_DICT.get(id)