# Rover Firmware für ESP32

> Automatisch generiert am 2025-05-04  
> Diese Dokumentation beschreibt die Firmwarestruktur, das I2C-Kommunikationsprotokoll und die Steuerung der Dynamixel AX-12 Servos.

---

## Inhaltsverzeichnis

- [Rover Firmware für ESP32](#rover-firmware-für-esp32)
  - [Inhaltsverzeichnis](#inhaltsverzeichnis)
  - [Projektübersicht](#projektübersicht)
  - [I2C-Kommunikation](#i2c-kommunikation)
    - [Allgemeine Struktur](#allgemeine-struktur)
    - [Command-IDs](#command-ids)
    - [SubCommand-IDs](#subcommand-ids)
    - [Kommunikationsbeispiel](#kommunikationsbeispiel)
  - [Dynamixel AX-12 Ansteuerung](#dynamixel-ax-12-ansteuerung)
    - [Servooperationen](#servooperationen)
    - [Raw-Werte und Winkelumrechnung](#raw-werte-und-winkelumrechnung)
  - [Hauptprogramm `main.cpp`](#hauptprogramm-maincpp)
  - [Hilfsfunktionen und Header](#hilfsfunktionen-und-header)
    - [`rover.h`](#roverh)
    - [`rover_utils.h`](#rover_utilsh)
    - [`i2c_helper.h`](#i2c_helperh)
  - [Anhang](#anhang)

---

## Projektübersicht

Die Firmware ermöglicht:
- Empfang und Verarbeitung von Steuerbefehlen über I2C im Binär
- Ansteuerung von Dynamixel AX-12 Servos
- Zugriff auf analoge und digitale IOs
- Zustandsabfrage des ESP32

---

## I2C-Kommunikation

### Allgemeine Struktur

Die Kommunikation zwischen Host (z. B. Raspberry Pi) und ESP32 erfolgt über I2C mit einem definierten Kommando-Protokoll.

Ein Datenpaket besteht aus:
- Header
- Command ID (1 Byte)
- SubCommand ID (1 Byte)
- Flags / Parameter (variabel)
- CRC8-Prüfsumme

### Command-IDs

| ID | Name                | Beschreibung                         |
|----|---------------------|--------------------------------------|
| 0  | `CMD_NONE`          | Kein Befehl                          |
| 1  | `CMD_SERVO_WRITE`   | Servo ansteuern                      |
| 2  | `CMD_SERVO_READ`    | Servo-Status lesen                   |
| 3  | `CMD_DIGITAL_WRITE` | Digitalen Pin setzen                |
| 4  | `CMD_DIGITAL_READ`  | Digitalen Pin lesen                 |
| 5  | `CMD_ANALOG_WRITE`  | Analogen Wert setzen                |
| 6  | `CMD_ANALOG_READ`   | Analogen Wert lesen                 |
| 7  | `CMD_ESP32_STATE`   | Zustandsdaten des ESP32 abfragen    |
| 8  | `CMD_MAX`           | Marker für maximale ID              |

### SubCommand-IDs

| ID | Name                        | Beschreibung                         |
|----|-----------------------------|--------------------------------------|
| 0  | `SCMD_NONE`                 | Kein Unterbefehl                     |
| 1  | `SCMD_SERVO_SPEED`          | Geschwindigkeit setzen               |
| 2  | `SCMD_SERVO_POSITION`       | Position setzen                      |
| 3  | `SCMD_SERVO_SPEED_POSITION` | Speed & Position kombinieren         |
| 4  | `SCMD_SERVO_TORQUE_ENABLE`  | Torque aktivieren / deaktivieren     |
| 5  | `SCMD_SERVO_LED_ON`         | LED am Servo aktivieren              |

### Kommunikationsbeispiel

**Beispiel zum Setzen der Servoposition:**

```json
{
  "write": 1,
  "cmd": 1,
  "sub": 2,
  "i16": 1,
  "fVal1": 0.5
}
```

---

## Dynamixel AX-12 Ansteuerung

Die Servos werden über die Bibliothek `Dynamixel2Arduino` angesteuert. Jeder Servo hat:
- Eine ID (0–253)
- Min-/Max-Raw-Werte
- Startposition

### Servooperationen

| Funktion                | Beschreibung                          |
|-------------------------|---------------------------------------|
| `dxl.setGoalPosition()` | Zielposition setzen                   |
| `dxl.setGoalVelocity()` | Geschwindigkeit setzen               |
| `dxl.torqueOn()`        | Motor aktivieren                     |
| `dxl.torqueOff()`       | Motor deaktivieren                   |
| `dxl.readPosition()`    | Aktuelle Position auslesen           |

### Raw-Werte und Winkelumrechnung

Die Umrechnung erfolgt in Hilfsfunktionen aus `rover_utils.h`. Jeder Servo hat individuell definierte MIN/MAX/START-Rawwerte.

---

## Hauptprogramm `main.cpp`

Das Hauptprogramm initialisiert:
- I2C als Slave
- Dynamixel-Servos
- Befehlsverarbeitung über ein `Command-Handler`-Array
- JSON-basierte und binäre Kommunikation

Die Hauptlogik basiert auf:
- Entgegennahme von I2C-Paketen
- Aufruf des zugehörigen Callbacks je nach `CommandID`

---

## Hilfsfunktionen und Header

### `rover.h`

Beinhaltet:
- Pins und Hardware-Konfiguration
- Aufzählungen für Servopositionen
- Definitionen für Servo-Grenzwerte

### `rover_utils.h`

Enthält:
- Umrechnung zwischen Rawwerten und Winkeln
- Funktionen zur Initialisierung und Regelung der Servos

### `i2c_helper.h`

Enthält:
- Struktur für binäre I2C-Pakete
- CRC-Berechnung
- Debug-Funktionen zur Analyse

---

## Anhang

Die Quellcodedateien befinden sich im Projektordner:

- `main.cpp`
- `rover.h`
- `rover_utils.h`
- `i2c_helper.h`