from ws2812ring import WS2812Ring
import time

leds = WS2812Ring(num_leds=7)

try:
    while True:
        print("Rot")
        leds.set_color(255, 0, 0)
        leds.show()
        time.sleep(1)

        print("Grün")
        leds.set_color(0, 255, 0)
        leds.show()
        time.sleep(1)

        print("Blau")
        leds.set_color(0, 0, 255)
        leds.show()
        time.sleep(1)

        print("Aus")
        leds.clear()
        time.sleep(1)

except KeyboardInterrupt:
    leds.clear()
    print("Beendet.")
