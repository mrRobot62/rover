import board
import busio
import digitalio
import time
import threading

#import adafruit_raspberry_pi5_neopixel_write as neopixel_write
import neopixel_spi

# | **Befehl / Eigenschaft**               | **Global / Einzel-LED** | **Beispiel**                                                      | **Info**                                                                 |
# |----------------------------------------|--------------------------|-------------------------------------------------------------------|--------------------------------------------------------------------------|
# | `NeoPixel_SPI(spi, n, brightness=…, auto_write=False)` | Init | `pixels = NeoPixel_SPI(board.SPI(), 28, brightness=0.5, auto_write=False)` | Erstellt Objekt mit `n` LEDs und globalem Helligkeitsfaktor  [oai_citation:0‡docs.circuitpython.org](https://docs.circuitpython.org/projects/neopixel_spi/en/stable/api.html?utm_source=chatgpt.com) |
# | `pixels.fill(color)`                  | Global                   | `pixels.fill((255,0,0))`                                           | Setzt alle LEDs gleichzeitig auf die angegebene Farbe  |
# | `pixels[i] = color`                   | Einzel-LED               | `pixels[3] = (0,255,0)`                                            | Setzt LED an Index `i` individuell              |
# | `pixels.show()`                       | Global                   | `pixels.show()`                                                    | Überträgt den Puffer per SPI an die LEDs       |
# | `pixels.deinit()`                     | Global                   | `pixels.deinit()`                                                  | Schaltet alle LEDs aus und gibt Ressourcen frei             |
# | `len(pixels)` / `pixels.n`            | Global                   | `count = len(pixels)`                                              | Liefert Anzahl der LEDs (read-only)                          |
# | `pixels.brightness = value`           | Global                   | `pixels.brightness = 0.2`                                          | Skaliert Helligkeit aller LEDs (0.0...1.0)   |
# | `pixels.pixel_order = neopixel_spi.GRB` | Global                 | `pixels = NeoPixel_SPI(..., pixel_order=neopixel_spi.GRB)`         | Setzt Farbkanal-Reihenfolge (z. B. RGB, GRB)                |
# | `pixels.frequency = ...`              | Global                   | `pixels.frequency = 3200000`                                       | Passt SPI-Frequenz (6.4 MHz default für 800 kHz LEDs)  |
# | `pixels.reset_time = ...`             | Global                   | `pixels.reset_time = 80e-6`                                        | Zeit für Reset zwischen `show()`-Rufen (Default ~80 µs)  |


"""
Hardware naher WS2812 Driver-Klasse

Sie sendet lediglich die Bits an die WS2812 LEDs die ON/OFF geschaltet werden sollen

Sie stellt nur rudimentäre WS2812 Funktionionen zur Verfügung

- fill : setzt alle LEDs (oder einzelne)
- blink : bringt eine LEDs (oder mehrere) in einen Blink-Modus (duration_on/duration_off)

"""

class WS2812SPI:
    def __init__(self, num_pixels=28, brightness=0.3, duration_on=0, duration_off=0, timeout=0, ledmask=0):
        self.num_pixels = num_pixels
        self.brightness = brightness
        self.duration_on = duration_on
        self.duration_off = duration_off
        self.timeout = timeout
        self.ledmask = ledmask

        # Datenausgang über SPI
        #self.pin = digitalio.DigitalInOut(board.MOSI)   # MOSI = PIN19
        #self.pin.direction = digitalio.Direction.OUTPUT

        # Aktueller LED-Zustand (GRB)
        self.pixels = [(0, 0, 0)] * self.num_pixels

        # Threading vorebereiten
        self._stop_event = threading.Event()
        self._fill_thread = None
        self._blink_thread = None
        self._run_thread = None

        self.driver = neopixel_spi.NeoPixel_SPI(
            board.SPI(),
            self.num_pixels,
            brightness=self.brightness,
            auto_write=False,
            frequency=6400000,     # für 800 kHz NeoPixel
            reset_time=80e-6       # 80 µs Pause nach jedem Frame
        )
        print (f"Pixels: {num_pixels},{bin(ledmask)}")

    def apply_brightness(self, color):
        """Helligkeit auf Farbwert anwenden (lokal, nicht global)."""
        return tuple(int(c * self.brightness) for c in color)


    def fill(self, color=(0, 0, 0), ledmask=None, timeout=0, **kwargs):
        """
        Setzt LEDs auf Farbe und ggf. nach timeout wieder auf OFF.
        """
        self.stop()
        ledmask = self.ledmask if ledmask is None else ledmask
        color = self.apply_brightness(color)

        def _fill_and_clear():
            for i in range(self.num_pixels):
                if ledmask & (1 << i):
                    self.driver[i] = color
                else:
                    self.driver[i] = (0, 0, 0)
            self.driver.show()

            if timeout > 0:
                start = time.time()
                while not self._stop_event.is_set():
                    if (time.time() - start) * 1000 >= timeout:
                        break
                    time.sleep(0.01)  # kurzes Polling
                # LEDs ausschalten nach Timeout
                for i in range(self.num_pixels):
                    self.driver[i] = (0, 0, 0)
                self.driver.show()

        self._fill_thread = threading.Thread(target=_fill_and_clear, daemon=True)
        self._fill_thread.start()

    def blink(self, color=(255, 165, 0), ledmask=None, duration_on=500, duration_off=500, timeout=0, **kwargs):
        """
        Nicht-blockierendes Blinken mit globalem Timeout.
        """
        self.stop()
        ledmask = self.ledmask if ledmask is None else ledmask
        color = self.apply_brightness(color)
        print(f"ws2812.BLINK color={color} mask={bin(ledmask)} DefaultMask: {hex(self.ledmask)}")

        def _blink_loop():
            start = time.time()
            while not self._stop_event.is_set():
                if timeout > 0 and (time.time() - start) * 1000 >= timeout:
                    break

                # ON
                for i in range(self.num_pixels):
                    if ledmask & (1 << i):
                        self.driver[i] = color
                    else:
                        self.driver[i] = (0, 0, 0)
                self.driver.show()
                if self._wait_or_stop(duration_on):
                    break

                # OFF
                for i in range(self.num_pixels):
                    self.driver[i] = (0, 0, 0)
                self.driver.show()
                if self._wait_or_stop(duration_off):
                    break

            # Nach Timeout: alle LEDs sicher aus
            for i in range(self.num_pixels):
                self.driver[i] = (0, 0, 0)
            self.driver.show()

        self._blink_thread = threading.Thread(target=_blink_loop, daemon=True)
        self._blink_thread.start()

    def run(self, color=(0, 255, 0), ledmask=None, duration_on=200, duration_off=200, timeout=100, **kwargs):
        """
        Lauflicht-Effekt: LEDs der Maske nacheinander AN/AUS mit Verzögerung.

        :param color: Farbe der leuchtenden LED
        :param ledmask: Bitmaske der aktiven LEDs
        :param duration_on: Wie lange LED an ist (ms)
        :param duration_off: Wie lange LED danach aus bleibt (ms)
        :param timeout: Wartezeit zwischen Start der nächsten LED (ms)
        """
        self.stop()
        ledmask = self.ledmask if ledmask is None else ledmask
        color = self.apply_brightness(color)

        # Positionen der aktiven LEDs extrahieren
        active_indices = [i for i in range(self.num_pixels) if ledmask & (1 << i)]
        print(f"ws2812.RUN active LEDs: {active_indices}")

        def _run_loop():
            while not self._stop_event.is_set():
                for i in active_indices:
                    if self._stop_event.is_set():
                        break

                    # LED i AN
                    for j in range(self.num_pixels):
                        self.driver[j] = color if j == i else (0, 0, 0)
                    self.driver.show()

                    if self._wait_or_stop(duration_on):
                        return

                    # LED i AUS
                    self.driver[i] = (0, 0, 0)
                    self.driver.show()

                    if self._wait_or_stop(duration_off):
                        return

                    # Wartezeit bis nächste LED beginnt
                    if self._wait_or_stop(timeout):
                        return

            # Alles aus am Ende
            for i in active_indices:
                self.driver[i] = (0, 0, 0)
            self.driver.show()

        self._run_thread = threading.Thread(target=_run_loop, daemon=True)
        self._run_thread.start()


    def _wait_or_stop(self, duration_ms):
        """
        Hilfsfunktion: wartet Dauer (ms), bricht aber ab wenn Stop-Event gesetzt wird.
        """
        end = time.time() + duration_ms / 1000.0
        while time.time() < end:
            if self._stop_event.is_set():
                return True
            time.sleep(0.01)
        return False


    def setBrightness(self, brightness=0.3):
        """
        Skaliert die globale Helligkeit.
        """
        print (f"ws2812.brightness({brightness})")
        self.driver.brightness=brightness

    def clear(self):
        print (f"ws2812.clear((0,0,0))")
        self.fill()


    def stop(self):
        """
        Stoppt ggf. laufende Blink- oder Pattern-Threads.
        """
        self._stop_event.set()
        for thread in [self._fill_thread, self._blink_thread, self._run_thread]:
            if thread and thread.is_alive():
                thread.join()
        self._stop_event.clear()

