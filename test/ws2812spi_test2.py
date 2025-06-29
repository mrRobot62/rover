from ws2812spi_driver import WS2812SPI
import time

leds = WS2812SPI(
    num_pixels=7, 
    brightness=0.5,
    ledmask=0xFFFFFFFF)

try:
    start = time.time()
    print(f"RUN2 - mit Timeout 3sek,  - {start}")
    leds.run(color=(0, 100, 255), ledmask=0b11111110, duration_on=20, duration_off=20, timeout=40)
    print(f"RUN2 - mit Timeout  - {time.time()- start}")


    time.sleep(10)
    leds.stop()
    leds.clear()

    print("ALLE Leds - ohne Timeout 3sek, dann stop()")
    leds.fill((30,128, 150), ledmask=0b1111111)
    time.sleep(3)
    leds.stop()

    start = time.time()
    print(f"LEDON - mit Timeout 3sek, kein expliziter stop() aufruf - {start}")
    leds.fill((200,128, 50), timeout=3000, ledmask=0b0101011)
    print(f"LEDON - mit Timeout 3sek, kein expliziter stop() aufruf - {time.time()- start}")

    time.sleep(6)
    leds.stop()

    start = time.time()
    print(f"BLINK1 - mit Timeout 3sek,  - {start}")
    leds.blink((50,128, 200), timeout=3000, ledmask=0b1110001)
    print(f"BLINK1 - mit Timeout  - {time.time()- start}")

    time.sleep(10)
    start = time.time()
    print(f"BLINK2 - mit Timeout 3sek,  - {start}")
    leds.blink((50,10, 200), timeout=3000, duration_off=200, duration_on=200, ledmask=0b1111110)
    print(f"BLINK2 - mit Timeout  - {time.time()- start}")

    time.sleep(10)
    start = time.time()
    print(f"RUN1 - mit Timeout 3sek,  - {start}")
    leds.run(color=(0, 100, 255), ledmask=0b11111110, duration_on=150, duration_off=15, timeout=100)
    print(f"RUN1 - mit Timeout  - {time.time()- start}")

    time.sleep(10)
    start = time.time()
    print(f"RUN2 - mit Timeout 3sek,  - {start}")
    leds.run(color=(0, 100, 255), ledmask=0b11111110, duration_on=20, duration_off=20, timeout=50)
    print(f"RUN2 - mit Timeout  - {time.time()- start}")


    time.sleep(10)
    print("Fertig – ausschalten")
    leds.stop()

except KeyboardInterrupt:
    print("Abbruch – ausschalten")
    leds.clear()
    leds.stop()
