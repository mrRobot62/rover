import board
import busio
import digitalio
import time
import threading

import adafruit_raspberry_pi5_neopixel_write as neopixel_write


class WS2812():
    def __init__(self, num_pixels=28, brightness=0.3, duration_on=0, duration_off=0, timeout=0):
        self.num_pixels = num_pixels
        self.brightness = brightness
        self.duration_on = duration_on
        self.duration_off = duration_off
        self.timeout = timeout

        # Datenausgang über SPI
        self.pin = digitalio.DigitalInOut(board.MOSI)   # MOSI = PIN19
        self.pin.direction = digitalio.Direction.OUTPUT

        # Aktueller LED-Zustand (GRB)
        self.pixels = [(0, 0, 0)] * self.num_pixels

        # Threading vorebereiten
        self._stop_event = threading.Event()
        self._pattern_thread = None
        
    #
    # allen methoden wird grundsätzlich in **kwargs die Parameter duration und timeout mit übergeben

    def apply_brightness(self, color):
        r, g, b = color
        return (
            int(r * self.brightness),
            int(g * self.brightness),
            int(b * self.brightness),
        )

    def clear(self, **kwargs):
        """ setzt alle LEDs auf OFF"""
        self.fill((0, 0, 0))

    def stop(self):
        """Stoppt einen laufenden Thread (Pattern)"""
        self._stop_event.set()
        if self._pattern_thread and self._pattern_thread.is_alive():
            self._pattern_thread.join()
        self._stop_event.clear()

    def fill(self, color=(0,0,0), **kwargs):
        print (f"LED {color}")
        self.pixels = [self.apply_brightness(color)] * self.num_pixels
        self.__show()

    def blink(self, direction=None, duration_on=500, duration_off=500, timeout=0, color=(255, 255, 0), **kwargs):
        """
        Beachten: ist eine nicht-blockiernde Methode und nutzt Threading.
        ein einfaches time.sleep() ist blockierend und das Node läuft nicht weiter und die ros-loop wird 
        ebenfalls blockiert
        
        """
        print("BLINK {direction}")
        self.stop()

        def _blink_loop():
            start_time = time.time()        
            while not self._stop_event.is_set():
                if timeout > 0 and (time.time() - start_time) * 1000 >= timeout:
                    break

                self.fill(color)
                time.sleep(duration_on / 1000)

                self.clear()
                time.sleep(duration_off / 1000)

            self.clear()

        self._pattern_thread = threading.Thread(target=_blink_loop, daemon=True)
        self._pattern_thread.start()


    def __show(self):
        """
        die Methode bildet ein ByteArray basieren auf das pixel-array und sendet es über MOSI 
        and die WS2812 LEDs

        """
        data = bytearray()
        for r, g, b in self.pixels:
            data += bytes([g, r, b])  # WS2812 verwenden GRB
        neopixel_write.neopixel_write(self.pin, data)

    

    
    

