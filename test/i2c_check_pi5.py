#!/usr/bin/env python3
import subprocess
import os
import sys

I2C_BUS = "/dev/i2c-1"
ESP32_ADDR_HEX = "12"  # Adresse 0x12 (ohne 0x)

def check_i2c_dev_module():
    print("Prüfe Kernel-Modul 'i2c_dev'...")
    result = subprocess.run(["lsmod"], capture_output=True, text=True)
    if "i2c_dev" in result.stdout:
        print("✓ i2c_dev Modul geladen")
    else:
        print("❌ i2c_dev Modul nicht geladen")
        print("Lade Modul temporär...")
        subprocess.run(["sudo", "modprobe", "i2c_dev"])
        print("✓ Modul geladen")

def check_i2c_device():
    print(f"Prüfe I2C Device {I2C_BUS}...")
    if os.path.exists(I2C_BUS):
        print(f"✓ {I2C_BUS} vorhanden")
    else:
        print(f"❌ {I2C_BUS} nicht vorhanden")
        sys.exit(1)

def check_all_i2c_busses():
    print("Gefundene I2C Busse:")
    os.system("ls /dev/i2c-*")

def run_i2cdetect():
    print(f"Starte i2cdetect auf {I2C_BUS}...")
    result = subprocess.run(["i2cdetect", "-y", "1"], capture_output=True, text=True)
    print(result.stdout)
    if ESP32_ADDR_HEX in result.stdout:
        print(f"✓ ESP32 gefunden bei Adresse 0x{ESP32_ADDR_HEX}")
    else:
        print(f"❌ Kein Gerät bei Adresse 0x{ESP32_ADDR_HEX} gefunden")

def main():
    print("=== I2C Check für Raspberry Pi 5 ===")
    check_i2c_dev_module()
    check_i2c_device()
    check_all_i2c_busses()
    run_i2cdetect()
    print("=== Check abgeschlossen ===")

if __name__ == "__main__":
    main()