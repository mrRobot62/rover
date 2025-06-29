import time
import threading
import board
import neopixel_spi

class WS2812SPI:
    def __init__(self, num_pixels=28, brightness=0.3):
        self.num_pixels = num_pixels
        self.brightness = brightness
        self.pixels = [(0, 0, 0)] * self.num_pixels

        # SPI-basiertes NeoPixel-Objekt
        self.driver = neopixel_spi.NeoPixel_SPI(
            board.SPI(),
            self.num_pixels,
            brightness=self.brightness,
            auto_write=False,
            frequency=6400000,    # 6.4 MHz für 800 kHz WS2812
            reset_time=80e-6      # 80 µs Pause
        )

        # Thread-Steuerung für Blinkmuster
        self._pattern_thread = None
        self._stop_event = threading.Event()

    def apply_brightness(self, color):
        """Helligkeit auf Farbwert anwenden (lokal, nicht global)."""
        return tuple(int(c * self.brightness) for c in color)

    def fill(self, color=(0, 0, 0), ledmask=0xFFFFFFFF, **kwargs):
        """
        Setzt LEDs gemäß ledmask auf Farbe. Default: alle.
        """
        print(f"fill() → color={color}, ledmask={bin(ledmask)}")
        color = self.apply_brightness(color)
        for i in range(self.num_pixels):
            self.driver[i] = color if ledmask & (1 << i) else (0, 0, 0)
        self.driver.show()

    def clear(self):
        """Alle LEDs ausschalten."""
        self.driver.fill((0, 0, 0))
        self.driver.show()

    def stop(self):
        """Aktives Pattern stoppen (z. B. blink)."""
        if self._pattern_thread and self._pattern_thread.is_alive():
            self._stop_event.set()
            self._pattern_thread.join()
            self._stop_event.clear()
            self.clear()

    def blink(self, color=(255, 165, 0), ledmask=0xFFFFFFFF, duration_on=500, duration_off=500, timeout=0, **kwargs):
        """
        Blinkmuster (nicht blockierend) auf LED-Bitmaske.
        """
        print(f"blink() → color={color}, mask={bin(ledmask)}")
        self.stop()

        def _blink_loop():
            start = time.time()
            while not self._stop_event.is_set():
                if timeout > 0 and (time.time() - start) * 1000 >= timeout:
                    break

                self.fill(color=color, ledmask=ledmask)
                time.sleep(duration_on / 1000)

                self.fill(color=(0, 0, 0), ledmask=ledmask)
                time.sleep(duration_off / 1000)

            self.clear()

        self._pattern_thread = threading.Thread(target=_blink_loop, daemon=True)
        self._pattern_thread.start()
