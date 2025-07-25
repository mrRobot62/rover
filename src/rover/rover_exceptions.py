# rover/src/rover/exceptions.py

class RoverException(Exception):
    """Basisklasse für alle Rover-bezogenen Fehler."""
    pass

class SensorTimeoutException(RoverException):
    """Sensor hat innerhalb des erwarteten Zeitraums keine Daten geliefert."""
    def __init__(self, sensor_name: str, timeout: float):
        super().__init__(f"Sensor '{sensor_name}' hat nach {timeout}s keine Antwort gegeben.")

class I2CCommunicationError(RoverException):
    """Fehler bei der Kommunikation mit einem I2C-Gerät."""
    def __init__(self, address: int, message: str = ""):
        msg = f"I2C-Fehler bei Adresse 0x{address:02X}"
        if message:
            msg += f": {message}"
        super().__init__(msg)

class InvalidLEDPatternException(RoverException):
    """Ein unbekanntes LED-Muster wurde angefordert."""
    def __init__(self, pattern_id: int):
        super().__init__(f"Ungültiges LED-Pattern: {pattern_id}")


class ServiceNotAvailableException(RoverException):
    """Ein geforderter Service oder der ServiceChannel ist nicht verfügbar"""
    def __init__(self, service: str):
        super().__init__(f"Service / Service-Channel {service} nicht verfügbar")


class NodeNotAvailableException(RoverException):
    """Ein geforderter Service oder der ServiceChannel ist nicht verfügbar"""
    def __init__(self, service: str):
        super().__init__(f"Service / Service-Channel {service} nicht verfügbar")