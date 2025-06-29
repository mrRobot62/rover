from ws2812spi_driver import WS2812SPI
import time

leds = WS2812SPI(
    num_pixels=7, 
    brightness=0.5,
    ledmask=0xFFFFFFFF)

try:
    print("Zeige Farben (rot → grün → blau)")
    leds.fill((255, 0, 0), ledmask=0b1111111)
    time.sleep(1)

    leds.fill((0, 255, 0))
    time.sleep(1)

    leds.fill((0, 0, 255))
    time.sleep(1)

    print("Starte Blinkmuster (0, 128, 255)")
    leds.blink(
        color=(0, 128, 255), 
        duration_on=250, 
        duration_off=250, timeout=2000,
        ledmask=0b0001
        )
    time.sleep(3)
    leds.blink(
        color=(0, 128, 255), 
        duration_on=250, 
        duration_off=250, timeout=2000,
        ledmask=0b0010
        )
    time.sleep(2)

    leds.blink(
        color=(0, 128, 255), 
        duration_on=250, 
        duration_off=250, timeout=2000,
        ledmask=0b0100
        )
    time.sleep(2)
    leds.blink(
        color=(0, 128, 255), 
        duration_on=250, 
        duration_off=250, timeout=2000,
        ledmask=0b1000
        )
    time.sleep(2)


    print("Starte Blinkmuster (orange, 3 Sekunden)")
    leds.blink(
        color=(30, 128, 200), 
        duration_on=500, 
        duration_off=250, timeout=2000)
    time.sleep(2)

    print("Starte Blinker")
    leds.blink(
        color=(255, 128, 0), 
        duration_on=800, 
        duration_off=300, timeout=5000,
        ledmask=0b1000111
        )
    time.sleep(5)

    print("Fertig – ausschalten")
    leds.clear()

except KeyboardInterrupt:
    print("Abbruch – ausschalten")
    leds.clear()
    leds.stop()
