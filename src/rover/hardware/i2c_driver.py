
from smbus2 import SMBus, i2c_msg
import smbus2


"""
i2c_driver.py

Dieses Modul stellt grundlegende I2C-Kommunikationsfunktionen bereit, um Datenpakete 
zwischen einem Raspberry Pi (Host) und einem ESP32-Mikrocontroller auszutauschen. 
Die Kommunikation erfolgt über ein eigenes binäres Protokoll mit Header, Payload und CRC8.
"""

# Konstanten zur Bus- und Adresskonfiguration
I2C_DEVICE = "/dev/i2c-"      # Pfad zum I2C-Device auf Linux
I2C_BUS_ID = 1             # I2C-Busnummer (i.d.R. 1 für RPi)

class I2CBus:
    """
    Singleton-Wrapper für den I2C-Bus. Stellt sicher, dass nur eine Bus-Instanz existiert.

    smbus2.SMBus – Methodenübersicht

    | Methode                               | Beschreibung                                                      |
    |---------------------------------------|-------------------------------------------------------------------|
    | open(bus)                             | Öffnet den angegebenen I2C-Bus                                    |
    | close()                               | Schließt die Verbindung zum I2C-Bus                               |
    | enable_pec(enable=True)               | Aktiviert oder deaktiviert Packet Error Checking (PEC)            |
    | write_quick(addr)                     | Führt einen "Quick Write" aus                                     |
    | read_byte(addr)                       | Liest ein einzelnes Byte vom Slave                                |
    | write_byte(addr, val)                 | Schreibt ein einzelnes Byte an den Slave                          |
    | read_byte_data(addr, reg)             | Liest ein Byte von einem bestimmten Register                      |
    | write_byte_data(addr, reg, val)       | Schreibt ein Byte in ein bestimmtes Register                      |
    | read_word_data(addr, reg)             | Liest ein 2-Byte-Wort vom angegebenen Register                    |
    | write_word_data(addr, reg, val)       | Schreibt ein 2-Byte-Wort in ein Register                          |
    | process_call(addr, reg, val)          | Sendet ein 16-Bit-Wort und erhält direkt eine Antwort             |
    | read_block_data(addr, reg)            | Liest bis zu 32 Bytes ab einem Register (SMBus Block Read)        |
    | write_block_data(addr, reg, data).    | Schreibt bis zu 32 Bytes ab einem Register (SMBus Block Write)    |
    | block_process_call(addr, reg, data).  | Sendet Block und erhält Block-Antwort (SMBus 2.0)                 |
    | read_i2c_block_data(addr, reg, len).  | Liest len Bytes wie bei I2C-Geräten                               |
    | write_i2c_block_data(addr, reg, data) | Schreibt Datenblock an I2C-Gerät                                  |
    | i2c_rdwr(msgs...)                     | Führt komplexe Lese-/Schreiboperationen (kombiniert) aus          |

    Hinweis: Fast alle Methoden besitzen ein optionales Argument 'force', um die Adressverwendung zu erzwingen.
    """
        
    #_bus_instance = None
    __bus = {}
    @classmethod
    def getBus(cls, i2c_bus_id=I2C_BUS_ID):
        if i2c_bus_id in cls.__bus:
            return cls.__bus[i2c_bus_id]
        else:
            cls.__bus[i2c_bus_id] = smbus2.SMBus(i2c_bus_id)
            return cls.__bus[i2c_bus_id]
        
    @classmethod
    def pingSlave(cls, bus: smbus2.SMBus, slave_address: int) -> bool:
        """ 
        prüfen ob die Slave-Adresse im Bus vorhanden ist.

        @return True - Slave-Adresse gefunden - ok
        @return False - Slave-Adresse nicht vorhanden
        """
        try:
            bus.write_quick(slave_address)
            return True
        except OSError:
            return False

    @classmethod
    def pingSlave(cls, i2c_bus_id, slave_address: int) -> bool:
        """ """
        try:
            bus = I2CBus.getBus(i2c_bus_id=i2c_bus_id)
            bus.write_quick(slave_address)
            return True
        except OSError:
            return False

