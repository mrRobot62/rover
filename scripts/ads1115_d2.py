from smbus2 import SMBus
import time

# I2C-Adresse des ADS1115 (Standard ist 0x48; ggf. 0x49/0x4A/0x4B prüfen)
ADS1115_ADDRESS = 0x48

# Register
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG     = 0x01

# PGA/Gain Bits (siehe TI-Datenblatt)
# 0x0000 = ±6.144 V
# 0x0200 = ±4.096 V
# 0x0400 = ±2.048 V  (Default nach Power-On)
# ...
GAIN = 0x0000  # ±6.144 V (für direkte 5-V-Messung)

# Lookup für Vollskalaspannung
GAIN_VOLTAGE_MAP = {
    0x0000: 6.144,
    0x0200: 4.096,
    0x0400: 2.048,
    0x0600: 1.024,
    0x0800: 0.512,
    0x0A00: 0.256,
}

# Kanal → MUX Bits (Single-Ended gegen GND)
MUX_MAP = {
    0: 0x4000,
    1: 0x5000,
    2: 0x6000,
    3: 0x7000,
}

# Data Rate: 1600 SPS (TI: DR[2:0]=100 → 0x0080)
DATA_RATE_1600SPS = 0x0080

# Comparator disabled: 0x0003 (COMP_QUE = 11)
COMP_DISABLED = 0x0003

def build_config(channel):
    """Baue Config-Wort für Single-Shot-Messung auf gewünschtem Kanal."""
    if channel not in MUX_MAP:
        raise ValueError(f"Invalid channel {channel}")
    return (
        0x8000 |              # OS = 1 → Start single conversion
        MUX_MAP[channel] |    # Kanalwahl
        GAIN |                # PGA/Gain
        DATA_RATE_1600SPS |   # 1600 SPS
        0x0100 |              # MODE = 1 (Single-Shot/Power-Down)
        COMP_DISABLED         # Comparator aus
    )

def start_conversion(bus, channel):
    """Schreibt Config und startet Messung."""
    cfg = build_config(channel)
    bus.write_i2c_block_data(
        ADS1115_ADDRESS,
        ADS1115_REG_CONFIG,
        [(cfg >> 8) & 0xFF, cfg & 0xFF]
    )

def wait_conversion_ready(bus, timeout=0.01):
    """
    Warte, bis OS-Bit = 1 (Conversion complete) oder Timeout.
    Rückgabe: True=ok, False=timeout.
    """
    t0 = time.time()
    while True:
        data = bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONFIG, 2)
        # data[0] enthält High-Byte; OS ist Bit15 → MSB
        if data[0] & 0x80:  # OS == 1?
            return True
        if (time.time() - t0) > timeout:
            return False
        # kurze Pause (1 ms)
        time.sleep(0.001)

def read_conversion(bus):
    """Rohwert lesen, in signed 16-bit wandeln."""
    data = bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONVERSION, 2)
    raw = (data[0] << 8) | data[1]
    if raw & 0x8000:  # negativ
        raw -= 0x10000
    return raw

def raw_to_voltage(raw):
    fs = GAIN_VOLTAGE_MAP.get(GAIN, 4.096)
    return (raw / 32768.0) * fs

def read_channel(bus, channel, discard_first=False):
    """
    Messe einen Kanal im Single-Shot-Modus.
    Optional: ersten Wert nach Kanalwechsel verwerfen (gegen Ghosting).
    """
    # Start Messung
    start_conversion(bus, channel)
    wait_conversion_ready(bus)

    if discard_first:
        _ = read_conversion(bus)  # Dummy lesen
        # zweite Messung starten
        start_conversion(bus, channel)
        wait_conversion_ready(bus)

    raw = read_conversion(bus)
    return raw_to_voltage(raw)

def main():
    last_channel = None
    with SMBus(1) as bus:
        while True:
            values = []
            for ch in range(4):
                # Wenn Kanal wechselt, ersten Wert verwerfen (reduziert Ghosting bei floating Eingängen)
                discard = (last_channel is None) or (ch != last_channel)
                v = read_channel(bus, ch, discard_first=discard)
                v = 0.0 if v < 0 else v
                values.append(v)
                last_channel = ch
            print("CH0 {:.4f} V\tCH1 {:.4f} V\tCH2 {:.4f} V\tCH3 {:.4f} V".format(*values))
            time.sleep(1)

if __name__ == "__main__":
    main()
