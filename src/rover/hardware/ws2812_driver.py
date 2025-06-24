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

class WS2812():
    def __init__(self, num_pixels=28, brightness=0.3, duration_on=0, duration_off=0, timeout=0, ledmask=0):
        self.num_pixels = num_pixels
        self.brightness = brightness
        self.duration_on = duration_on
        self.duration_off = duration_off
        self.timeout = timeout
        self.ledmask = ledmask

        # Datenausgang über SPI
        self.pin = digitalio.DigitalInOut(board.MOSI)   # MOSI = PIN19
        self.pin.direction = digitalio.Direction.OUTPUT

        # Aktueller LED-Zustand (GRB)
        self.pixels = [(0, 0, 0)] * self.num_pixels

        # Threading vorebereiten
        self._stop_event = threading.Event()
        self._pattern_thread = None
        
        self.driver = neopixel_spi.NeoPixel_SPI(
            board.SPI(),
            self.num_pixels,
            brightness=self.brightness,
            auto_write=False,
            frequency=6400000,     # für 800 kHz NeoPixel
            reset_time=80e-6       # 80 µs Pause nach jedem Frame
        )

    def fill(self, color=(0,0,0), ledmask=None, **kwargs):
        """
        color setzt die Farbe, Default alle 0 = OFF, selbst ewnn ledmask <> 0 ist !
        ledmask gibt an welche LEDs gesetzt werden (default alle auf 0)
        Warum ist ledmask im Default alle Bits auf 1? 
        => Weil dann ein einfacher fill((200,200,50)) befehl ausreicht um alle LEDs auf einen Schlag anzuschalten
        """
        # print (f"ws2812.fill color: {color}, LEDMask: {bin(ledmask)}")
        # Global füllen — Sicherheitsreset für alle LEDs
        self.driver.fill((0, 0, 0))

        for i in range(self.num_pixels):
                if ledmask & (1 << i):
                    self.driver[i] = color
                else:
                    self.driver[i] = (0,0,0)

        # LEDs setzen
        self.driver.show()

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
        if self._pattern_thread and self._pattern_thread.is_alive():
            self._pattern_thread.join()
        # Reset Event für spätere Patterns
        self._stop_event.clear()

    def blink(self, color=(255,165,0), ledmask=None, duration_on=None, duration_off=None, timeout=None, **kwargs):
        """
        Nicht-blockierendes Blinken ausgewählter LEDs via Bitmaske.

        :param color: RGB-Tupel für "an"-Zustand
        :param ledmask: Welche LEDs blinken (1 = blink)
        :param duration_on/off: Zeiten in ms
        :param timeout: Gesamtdauer in ms (0 = unendlich)
        """
        print(f"ws2812.BLINK color={color} mask={bin(ledmask)}")
        self.stop()
        ledmask = self.ledmask if ledmask == None else ledmask
        print(f"ws2812.BLINK color={color} mask={bin(ledmask)} DefaultMask: {self.ledmask}")

        def _blink_loop():
            start = time.time()
            while not self._stop_event.is_set():
                if timeout > 0 and (time.time() - start) * 1000 >= timeout:
                    break

                # ON: nur LEDs laut Maske
                self.fill(color=color, ledmask=ledmask)
                time.sleep(duration_on / 1000)

                # OFF: 
                self.fill(color=(0,0,0), ledmask=0x0)
                time.sleep(duration_off / 1000)

            # Nach Timeout: sicher ausschalten
            self.fill(color=(0,0,0), ledmask=ledmask)

        self._pattern_thread = threading.Thread(target=_blink_loop, daemon=True)
        self._pattern_thread.start()
