import json
import time
import smbus2

from smbus2 import SMBus, i2c_msg

# I2C Konfiguration
I2C_BUS = 1  # Raspberry Pi 5 I2C Bus Nummer
ESP32_ADDR = 0x12  # I2C Adresse vom ESP32

# Commands als Enum
CMD_NONE = 0
CMD_WRITE_SERVO = 1
CMD_WRITE_CONFIG = 2
CMD_READ_SERVO_STATE = 3
CMD_READ_SYSTEM_INFO = 4
CMD_READ_IMU_DATA = 5
CMD_MAX = 6

# CRC8 Dallas/Maxim
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


# # Write-Command senden
# def send_write_command(bus, cmd_id, payload):
#     json_data = json.dumps(payload).encode("utf-8")
#     crc_value = crc8(json_data)
#     send_data = json_data + bytes([crc_value])

#     print(f"Sende WRITE Command {cmd_id} mit CRC {crc_value:02X}")
#     bus.write_i2c_block_data(ESP32_ADDR, 0, list(send_data))


def send_write_command(bus, cmd_id, payload):
    json_data = json.dumps(payload).encode("utf-8")
    crc_value = crc8(json_data)
    send_data = json_data + bytes([crc_value])

    print(f"Sende WRITE Command {cmd_id} mit CRC {crc_value:02X}")

    msg = i2c_msg.write(ESP32_ADDR, send_data)
    bus.i2c_rdwr(msg)


# Read-Command senden und Antwort empfangen
def send_read_command(bus, cmd_id, rx_json):
    payload = {
        "r": cmd_id
    }
    json_data = json.dumps(payload).encode("utf-8")
    crc_value = crc8(json_data)
    send_data = json_data + bytes([crc_value])

    print(f"Sende READ Command {cmd_id} mit CRC {crc_value:02X}")
    bus.write_i2c_block_data(ESP32_ADDR, 0, list(send_data))

    time.sleep(0.05)

    read_data = bus.read_i2c_block_data(ESP32_ADDR, 0, 128)
    raw_data = bytes(read_data).rstrip(b'\x00')

    print("Antwort (raw):", raw_data.decode(errors="ignore"))

    try:
        response = json.loads(raw_data.decode("utf-8"))
        rx_json.clear()
        rx_json.update(response)
        print("Antwort (json):", rx_json)
    except Exception as e:
        print("Fehler beim JSON Parsen:", e)


# Platzhalter-Funktionen für Reads
def read_servo_state(rx_json):
    print("Lese Servo State...")
    print(rx_json)

def read_system_info(rx_json):
    print("Lese System Info...")
    print(rx_json)

def read_imu_data(rx_json):
    print("Lese IMU Data...")
    print(rx_json)


def main():
    bus = smbus2.SMBus(I2C_BUS)

    try:
        while True:
            print("\nBefehls-Auswahl:")
            print("1 = Write Servo")
            print("2 = Write Config")
            print("3 = Read Servo State")
            print("4 = Read System Info")
            print("5 = Read IMU Data")
            print("q = Beenden")

            choice = input(">> ")

            if choice == "1":
                # payload = {
                #     "w": CMD_WRITE_SERVO,
                #     "dx": [500,300,0,0],
                #     "i": [0,0,0,0]
                # }

                # json_data = json.dumps(payload).encode("utf-8")


                # print(f"Payload({json_data}) Len: {len(json_data)}")
                # send_write_command(bus, CMD_WRITE_SERVO, payload)
                try:
                    speed = float(input("Speed [-1.0 bis 1.0]: "))
                    curve = float(input("Curve [-1.0 bis 1.0]: "))

                    # Begrenzen falls nötig
                    speed = max(-1.0, min(1.0, speed))
                    curve = max(-1.0, min(1.0, curve))

                    iSpeed = int(speed * 100);
                    iCurve = int(curve * 100);

                    payload = {
                        "w": CMD_WRITE_SERVO,
                        "dx": [iSpeed, iCurve,0,0],
                        "i" : [0,0,0,0]
                    }

                    send_write_command(bus, CMD_WRITE_SERVO, payload)

                except ValueError:
                    print("Ungültige Eingabe, bitte Float-Wert angeben.")
            elif choice == "2":
                payload = {
                    "w": CMD_WRITE_CONFIG,
                    "dx": [500,300,0,0],
                    "i": [0,0,0,0]
                }
                send_write_command(bus, CMD_WRITE_CONFIG, payload)

            elif choice == "3":
                rx_json = {}
                send_read_command(bus, CMD_READ_SERVO_STATE, rx_json)
                read_servo_state(rx_json)

            elif choice == "4":
                rx_json = {}
                send_read_command(bus, CMD_READ_SYSTEM_INFO, rx_json)
                read_system_info(rx_json)

            elif choice == "5":
                rx_json = {}
                send_read_command(bus, CMD_READ_IMU_DATA, rx_json)
                read_imu_data(rx_json)

            elif choice == "q":
                break

            else:
                print("Ungültige Eingabe.")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Beendet.")

    bus.close()


if __name__ == "__main__":
    main()