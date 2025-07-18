import time

""" 
ACS712Driver

Diese Klasse liest über das ads_driver Objekt die voltage auch dem channel und berechnet den aktuellen
Stromverbrauch.

Die Klasse wird über den I2CNode genutzt. Der ADC-Node publiziert dien aktuellen Stromverbrauch in einem Topic

"""
class ACS712Driver:

    def __init__(self, ads_driver, channel:int=1, zero_offset=2.5, sensitivity=0.185):
        """
        :param ads_driver: ADS1115Driver-Objekt
        :param zero_offset: Spannung in Volt bei 0 A (kalibriert)
        :param sensitivity: Empfindlichkeit in V/A (z. B. 0.185 für 5A-Modul)
        """
        self.ads_driver = ads_driver
        self.zero_offset = zero_offset
        self.sensitivity = sensitivity
        self.channel = channel

    def read_voltage(self, samples=10, delay=0.01):
        """Liest die durchschnittliche Spannung vom ADC. Im Default 10 Samples mit einer Verzögerung von 0.01ms (10ms) und dann den Durchschnitt"""
        voltages = []
        for _ in range(samples):
            voltage = self.ads_driver.read_voltage(self.channel)
            voltages.append(voltage)
            time.sleep(delay)
        return sum(voltages) / len(voltages)

    #
    #
    # Eigentlich in ein time.sleep() nicht besonders effektiv, da es ein blockierender aufruf ist.
    # eine Verbesserte Version wäre ein eigenständiger Thread, bedarf aber einem höheren Overhead und Verwaltung
    #
    def read_current(self, samples=10, delay=0.01):
        """Berechnet den Strom in Ampere basierend auf ADC-Spannung."""
        voltage = self.read_voltage(samples, delay)
        current = (voltage - self.zero_offset) / self.sensitivity
        return round(current, 3)

    def calibrate_zero(self, samples=100, delay=0.01):
        """Kalibriert den Nullpunkt (bei 0 A angeschlossen)."""
        self.zero_offset = self.read_voltage(samples, delay)
        return self.zero_offset



# class depr_ACS712:
#     def __init__(self, ads_driver, zero_offset=2.5, sensitivity=0.185, sample_interval=0.01, averaging_window=10):
#         """
#         :param ads_driver: Objekt mit Methode .read_voltage() → float (Volt)
#         :param zero_offset: Kalibrierte Spannung bei 0 A
#         :param sensitivity: Empfindlichkeit in V/A
#         :param sample_interval: Zeit zwischen ADC-Lesungen (in Sekunden)
#         :param averaging_window: Anzahl Spannungswerte zur Mittelung
#         """
#         self.ads_driver = ads_driver
#         self.zero_offset = zero_offset
#         self.sensitivity = sensitivity
#         self.sample_interval = sample_interval
#         self.averaging_window = averaging_window

#         self._voltage = 0.0
#         self._voltage_buffer = []
#         self._lock = threading.Lock()
#         self._running = False
#         self._thread = None

#     def start(self):
#         if not self._running:
#             self._running = True
#             self._thread = threading.Thread(target=self._update_loop, daemon=True)
#             self._thread.start()

#     def stop(self):
#         if self._running:
#             self._running = False
#             if self._thread:
#                 self._thread.join()
#                 self._thread = None

#     def _update_loop(self):
#         while self._running:
#             try:
#                 voltage = self.ads_driver.read_voltage()
#                 with self._lock:
#                     self._voltage_buffer.append(voltage)
#                     if len(self._voltage_buffer) > self.averaging_window:
#                         self._voltage_buffer.pop(0)
#                     self._voltage = sum(self._voltage_buffer) / len(self._voltage_buffer)
#             except Exception as e:
#                 print(f"[ACS712] Fehler beim Lesen: {e}")
#             time.sleep(self.sample_interval)

#     def read_voltage(self):
#         with self._lock:
#             return self._voltage

#     def read_current(self):
#         voltage = self.read_voltage()
#         current = (voltage - self.zero_offset) / self.sensitivity
#         return round(current, 3)

#     def calibrate_zero(self):
#         """Kalibriere den Offset bei ruhendem Strom (z. B. direkt nach Start)."""
#         time.sleep(self.sample_interval * self.averaging_window)
#         self.zero_offset = self.read_voltage()
#         return self.zero_offset