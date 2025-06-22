import time
import json
from smbus2 import SMBus, i2c_msg

# I2C Einstellungen
I2C_BUS = 1
I2C_ADDRESS = 0x12  # Deine ESP32-Slave-Adresse

# CRC8 – Dallas/Maxim-Variante
def calc_crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

# Beispielbefehl – entspricht CMD_WRITE_SERVO_DRIVE
command = {
    "w": 1,  # Command-ID: CMD_WRITE_SERVO_DRIVE
    "dx": [50, 0, 0, 0],  # Geschwindigkeit / Lenkung als int
    "i": [0, 0, 0, 0]
}

# JSON codieren + CRC anhängen
json_bytes = json.dumps(command, separators=(",", ":")).encode("utf-8")
crc_byte = bytes([calc_crc8(json_bytes)])
payload = json_bytes + crc_byte

# Ausgabe zu Testzwecken
print(f"→ JSON: {json_bytes}")
print(f"→ CRC8: {crc_byte.hex()}")
print(f"→ Sende {len(payload)} Bytes an I2C-Adresse 0x{I2C_ADDRESS:02X}")

# Übertragung per I2C
try:
    with SMBus(I2C_BUS) as bus:
        msg = i2c_msg.write(I2C_ADDRESS, payload)
        bus.i2c_rdwr(msg)
        print("✅ Übertragung erfolgreich.")
except Exception as e:
    print(f"❌ Fehler bei I2C-Schreibvorgang: {e}")