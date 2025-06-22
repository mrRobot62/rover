# 📦 ESP32 Rover Firmware – Modul-Dokumentation

Diese Dokumentation beschreibt die wichtigsten Bestandteile der Firmware des ESP32-gestützten Rovers. Sie deckt den Aufbau und die Aufgaben der Hauptmodule ab und schlägt Erweiterungen vor.

---

# `main.cpp`

## 📝 Funktion

Dieses Modul ist der Einstiegspunkt des Programms. Es initialisiert:
- die serielle Kommunikation,
- den Dynamixel-Bus,
- die I2C-Kommunikation mit dem Host (z. B. Raspberry Pi),
- und das Callback-System für Kommandoverarbeitung.

Die `loop()`-Funktion verarbeitet zyklisch I2C-Nachrichten und delegiert sie an die passenden Handler.

## 🔍 Struktur

- `setup()`: Initialisierung
- `loop()`: Hauptverarbeitungsschleife
- Registrierung aller unterstützten Kommandos (`CMD_SERVO_WRITE`, `CMD_ANALOG_READ` etc.)

## 💡 Erweiterungsvorschläge

- Logging verbessern mit Zeitstempeln
- Erweiterbare Kommando-Tabelle auslagern
- Sleep/Power-Management ergänzen

---

# `rover.h`

## 📝 Funktion

Definiert zentrale Hardware-Konstanten und stellt grundlegende Initialisierung und Steuerung der Dynamixel-Servos bereit.

## 🔍 Struktur

- I2C- und Dynamixel-Konstanten (`DEFAULT_I2C_SDA`, `DXL_BROADCAST_ID`)
- Aliasnamen für verwendete Schnittstellen (`DXL_SERIAL`, `DEBUG_SERIAL`)
- `TDynaList` = Vektor-Typ zur Verwaltung dynamischer Servo-ID-Listen

## 💡 Erweiterungsvorschläge

- Kapselung in eine `RoverHardwareConfig`-Klasse
- Alle Hardwarewerte über `#define` → `constexpr` umstellen
- Kompatibilität mit mehreren Dynamixel-Bus-Konfigurationen

---

# `rover_utils.h`

## 📝 Funktion

Enthält nützliche Funktionen zur Servo-Steuerung und ID-Verwaltung.

## 🔍 Struktur

- `id_exists(...)`: prüft, ob eine Servo-ID in einer Liste enthalten ist
- `setServoFromAngle(...)`: setzt Servo-Position basierend auf einem Winkel
- `clamp(...)`: begrenzt Werte in einem definierten Wertebereich
- Hilfs-Makros wie `ARRAY_SIZE` und `SPEED_MULTIPLIER`

## 💡 Erweiterungsvorschläge

- Unit Tests für alle Hilfsfunktionen
- `setServoFromAngle()` um min/max Limits erweitern
- Umstellung auf constexpr/inline für Makros und utility-Funktionen

---

# ✅ Fazit

Die Firmwarestruktur ist gut modularisiert. Die Datei `main.cpp` bleibt übersichtlich, da sie ihre Logik an `rover.h` und `rover_utils.h` auslagert. Die Erweiterung über Kommandos und Handler ist durchdacht und zukunftsfähig.