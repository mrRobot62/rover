import board
import digitalio
import time
import neopixel_spi

def test_spi():
    try:
        # SPI-Initialisierung
        driver = neopixel_spi.NeoPixel_SPI(board.SPI(), 28, brightness=0.5, auto_write=False)
        driver.fill((255, 0, 0))  # Alle LEDs rot
        driver.show()  # Übertrage den Puffer an die LEDs
        print("SPI funktioniert korrekt")
    except Exception as e:
        print(f"Fehler bei SPI-Kommunikation: {e}")

test_spi()
