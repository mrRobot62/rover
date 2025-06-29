import time
import platform
import sys
import os

class WS2812Ring:
    def __init__(self, num_leds=7):
        self.num_leds = num_leds
        self.model = self.detect_pi_model()
        self.backend = None
        self.buffer = [(0, 0, 0)] * num_leds  # RGB-Werte
        self._init_backend()

    def detect_pi_model(self):
        try:
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().lower()
                if "raspberry pi 5" in model:
                    return "pi5"
                elif "raspberry pi" in model:
                    return "pi_other"
        except:
            pass
        return "unknown"

    def _init_backend(self):
        if self.model == "pi5":
            # Pi 5 – PIO-Treiber
            import board
            import digitalio
            import adafruit_raspberry_pi5_neopixel_write as neopixel

            pin = digitalio.DigitalInOut(board.MOSI)
            pin.direction = digitalio.Direction.OUTPUT

            self.backend = {
                "type": "pio",
                "pin": pin,
                "write": neopixel.neopixel_write
            }

        elif self.model == "pi_other":
            # Pi 4, 3, Zero – rpi_ws281x
            from rpi_ws281x import PixelStrip, Color

            LED_PIN = 18  # GPIO18 = Pin 12
            LED_FREQ_HZ = 800000
            LED_DMA = 10
            LED_BRIGHTNESS = 255
            LED_INVERT = False
            LED_CHANNEL = 0

            strip = PixelStrip(self.num_leds, LED_PIN, LED_FREQ_HZ,
                               LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
            strip.begin()

            self.backend = {
                "type": "ws281x",
                "strip": strip,
                "color": Color
            }

        else:
            raise RuntimeError("Nicht unterstütztes Gerät für WS2812Ring")

    def set_color(self, red, green, blue):
        """Setzt Farbe aller LEDs im internen Buffer"""
        self.buffer = [(red, green, blue)] * self.num_leds

    def clear(self):
        """Alle LEDs aus"""
        self.set_color(0, 0, 0)
        self.show()

    def show(self):
        """Buffer anzeigen"""
        if self.backend["type"] == "pio":
            data = bytearray()
            for r, g, b in self.buffer:
                # GRB-Format
                data += bytes([g, r, b])
            self.backend["write"](self.backend["pin"], data)

        elif self.backend["type"] == "ws281x":
            strip = self.backend["strip"]
            Color = self.backend["color"]
            for i, (r, g, b) in enumerate(self.buffer):
                strip.setPixelColor(i, Color(r, g, b))
            strip.show()
