from smbus2 import SMBus
import time

# I2C-Adresse des ADS1115 (Standard ist 0x48)
ADS1115_ADDRESS = 0x48

# Register
ADS1115_REG_CONVERSION = 0x00
ADS1115_REG_CONFIG     = 0x01

# Gain = +/- 4.096V (FSR), 1 bit = 125uV
GAIN = 0x0000

# Konfigurationsbits für Einzelmessung auf AIN0
def config_channel(channel):
    mux = {
        0: 0x4000,  # AIN0 vs GND
        1: 0x5000,  # AIN1 vs GND
        2: 0x6000,  # AIN2 vs GND
        3: 0x7000,  # AIN3 vs GND
        }.get(channel, 0x4000)

    config = (
        0x8000 |      # Start single conversion
        mux    |      # Channel selection
        GAIN   |      # Gain
        0x0100 |      # 1600 SPS
        0x0003        # Single-shot mode, disable comparator
    )
    return config

# def read_channel(bus, channel):
#     config = config_channel(channel)
#     # Konfiguration schreiben
#     bus.write_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])

#     # Warten auf Messung (mind. 1/1600s ≈ 0.625ms, zur Sicherheit 1ms)
#     time.sleep(0.001)

#     # Messwert lesen
#     data = bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONVERSION, 2)
#     raw = (data[0] << 8) | data[1]

#     # Vorzeichen beachten
#     if raw > 0x7FFF:
#         raw -= 0x10000

#     voltage = raw * 6.144 / 32768  # bei ±4.096 V Gain
#     time.sleep(0.01)
#     return voltage

def read_channel(bus, channel):
    config = config_channel(channel)
    # Konfiguration schreiben
    bus.write_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])

    # Warten bis Messung abgeschlossen ist (Bit 15 == 0)
    while True:
        status = bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONFIG, 2)
        if (status[0] & 0x80) == 0:  # Bit 15 (MSB) == 0 → Conversion ready
            break
        time.sleep(0.001)  # Warten 1 ms

    # Messwert lesen
    data = bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_REG_CONVERSION, 2)
    raw = (data[0] << 8) | data[1]
    if raw > 0x7FFF:
        raw -= 0x10000

    # Berechne Spannung
    voltage = raw * 6.144 / 32768  # GAIN = ±6.144 V
    return voltage


def main():
    with SMBus(1) as bus:
        while True:
            ch0 = read_channel(bus, 0)
            ch1 = read_channel(bus, 1)
            ch2 = read_channel(bus, 2)
            ch3 = read_channel(bus, 3)

            print(f"CH0 {ch0:.4}\tCH1 {ch1:.4}\tCH2 {ch2:.4}\tCH3 {ch3:.4}")
            time.sleep(1)

if __name__ == "__main__":
    main()
