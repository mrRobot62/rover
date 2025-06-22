import struct
from smbus2 import SMBus, i2c_msg
from enum import Enum

I2C_ADDRESS = 0x12
I2C_BUS = 1


class CommandID(Enum):
    NONE=0
    SERVO_WRITE=1      # Zur Nutzt unv Dynamixel-Servos
    SERVO_READ=2       # Status eines Servos auslesen
    DIGITAL_WRITE=3    # Digital-Pin Wert setzen (0/1)
    DIGITAL_READ=4     # Digital-Pin Wert auslesen (0/1)
    ANALOG_WRITE=5     # Analog-Pin Wert setzen (0..1023)
    ANALOG_READ=6      # Analog-Pin Wert auslesen (0..1023)
    ESP32_STATE=7       # Status des ESP32 auslesen

class SubCommandID(Enum):
    SCMD_NONE=0
    SCMD_SERVO_SPEED=1
    SCMD_SERVO_POSITION=2
    SCMD_SERVO_SPEED_POSITION=3
    SCMD_SERVO_TORQUE_ENABLE=4
    SCMD_SERVO_LED_ON=5

class DIGITALPINS(Enum):
    ONBOARD_LED=2
    LED1=19
    LED2=18
    IO1=5
    IO2=4
    IO3=15
    IO4=23
    ADC0=36
    ADC1=39
    ADC2=34
    ADC3=35

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x31 if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

def dataModulo(data_vals, factor=100):
    """
    Skaliert eine Liste numerischer Werte und konvertiert sie in uint16_t-kompatible Ganzzahlen.

    Jeder Wert in `data_vals` wird mit dem gegebenen `factor` multipliziert und anschließend
    per Modulo 65536 in den Wertebereich eines `uint16_t` (0..65535) überführt.

    Dies ist besonders nützlich für Binärprotokolle, bei denen negative Werte über das
    Zweierkomplement als unsigned übertragen werden müssen.

    Beispiel:
        input:  [-1.0, 0.5, 1.0]
        output: [65536 - 100 = 65436, 50, 100]

    Parameter:
    ----------
    data_vals : list of float or int
        Eine Liste mit Zahlenwerten (z. B. [-1.0, 0.5, 1.0]), die skaliert und in uint16_t 
        Bereich überführt werden sollen.

    factor : int, optional (default=100)
        Der Skalierungsfaktor, mit dem jeder Wert vor der Konvertierung multipliziert wird.
        Typische Werte: 100 für zwei Nachkommastellen Genauigkeit.

    Returns:
    --------
    list of int
        Eine neue Liste mit Werten im Bereich 0 bis 65535, geeignet für struct.pack("H").
    """
    return [int(val * factor) % 65536 for val in data_vals]    



def send_packet(cmd, subcmd=0, flags=0, data_vals=None):
    if data_vals is None:
        data_vals = [0] * 5

    header = 0xFEEF
    reserved = 0x00
    payload = struct.pack("<HBBB5HB", header, cmd, subcmd, flags, *data_vals, reserved)
    crc = crc8(payload)
    packet = payload + bytes([crc])

    print(f"→ Sende Paket: {[hex(b) for b in packet]}")

    try:
        with SMBus(I2C_BUS) as bus:
            msg = i2c_msg.write(I2C_ADDRESS, packet)
            bus.i2c_rdwr(msg)
        print("✅ Paket gesendet.\n")
    except Exception as e:
        print(f"❌ Fehler beim Senden: {e}\n")

# Platzhalterfunktionen für zukünftige Kommandos
def servo_read():
    print("[servo_read] Noch nicht implementiert.")

def digtal_read():
    print("[digtal_read] Noch nicht implementiert.")

def analog_write():
    print("[analog_write] Noch nicht implementiert.")

def analog_read():
    print("[analog_read] Noch nicht implementiert.")

def esp32_state():
    print("[esp32_state] Noch nicht implementiert.")


def digital_write():
    cmd = CommandID.DIGITAL_WRITE.value
    scmd = SubCommandID.SCMD_NONE.value
    print(f"→ DIGITAL_WRITE ({cmd}, {scmd})")

    e = input(" 1. GPIO-Pin   : ").strip()
    p1 = int(e) if e else 0
    e = input("    State (0/1) : ").strip()
    s1 = int(e) if e else 0
    
    e = input(" 2. GPIO-Pin   : ").strip()
    p2 = int(e) if e else 0
    e = input("    State (0/1) : ").strip()
    s2 = int(e) if e else 0

    if p1 not in (item.value for item in DIGITALPINS):
        print("ungültiger IO-PIN1, reset to 0")
        p1 = 0
    if p2 not in (item.value for item in DIGITALPINS):
        print("ungültiger IO-PIN2, reset to 0")
        p2 = 0
    
    data = [p1,p2,s1,s2,0]
    send_packet(cmd=int(cmd), subcmd=int(scmd), flags=0x00, data_vals=data)
    return True    

def servo_write():
    print("→ SERVO_WRITE:")
    cmd = 1
    val2 = 0
    scmd = input(" (1) Velocity oder (2) Steering oder (3) Beides : ").strip()
    if scmd not in ["1", "2", "3"]:
        print(" Ungültige Auswahl.\n")
        return False
    try: 
        if scmd == "1" or scmd == "2":
            val = float(input(" Wert im Bereich -1.0 bis 1.0: ").strip())

        if scmd == "3":
            val = float(input("Speed Wert im Bereich -1.0 bis 1.0: ").strip())
            val2 = float(input("Curve Wert im Bereich -1.0 bis 1.0: ").strip())
    
        if not -1.0 <= val <= 1.0:
            raise ValueError
        if not -1.0 <= val2 <= 1.0:
            raise ValueError
        
        if scmd == "1":
            data = [val, 0, 0, 0, 0]
        elif scmd == "2":
            data = [0, val, 0, 0, 0]
        elif scmd == "3":
            data = [val, val2, 0, 0, 0]
        
        data = dataModulo(data_vals=data, factor=100)
        print(f"    => {cmd}.{scmd} Data: {data}")
        send_packet(cmd=cmd, subcmd=int(scmd), flags=0x00, data_vals=data)
        return True
    
    except ValueError:
        print(" ❌ Ungültiger Wert.\n")
        return False

# Interaktive Schleife
def main():
    run = True
    while (run == True):
        print("Befehlsauswahl:")
        print("1 = SERVO_WRITE")
        print("2 = SERVO_READ")
        print("3 = DIGITAL_WRITE")
        print("4 = DIGITAL_READ")
        print("5 = ANALOG_WRITE")
        print("6 = ANALOG_READ")
        print("7 = ESP32_STATE")
        print("q = Beenden")

        choice = input("Auswahl: ").strip().lower()

        if choice == "1":
            run = servo_write()
        elif choice == "2":
            run = servo_read()
        elif choice == "3":
            run = digital_write()
        elif choice == "4":
            run = digtal_read()
        elif choice == "5":
            run = analog_write()
        elif choice == "6":
            run = analog_read()
        elif choice == "7":
            run = esp32_state()
        elif choice == "q":
            print("Programm beendet.")
            run = False
            break
        else:
            print("❌ Ungültige Eingabe.\n")
            run = False

if __name__ == "__main__":
    main()