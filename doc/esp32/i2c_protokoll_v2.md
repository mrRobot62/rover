# Übertragungsprotokoll PI <-> ESP32 Version V2

Das Protokoll muss aus Geschwindigkeitsgründen sehr kompakt und klein sein.
Versuche mit einer JSON-Struktur sind gescheitert, da es zu Fehlern gekommen ist.

- [Übertragungsprotokoll PI \<-\> ESP32 Version V2](#übertragungsprotokoll-pi---esp32-version-v2)
  - [PIN-OUT ESP32 - RaspberryPI](#pin-out-esp32---raspberrypi)
  - [📦 I²C-Paketstruktur (binäres Format, 17 Bytes Gesamt)](#-ic-paketstruktur-binäres-format-17bytes-gesamt)
  - [Feldname: Header `header`(2 Bytes)](#feldname-header-header2-bytes)
  - [Feldname: Commands `cmd` (1 Byte)](#feldname-commands-cmd-1-byte)
  - [Feldname: `SubCommmads` (1 Byte)](#feldname-subcommmads-1-byte)
  - [Feldname: `flags` (1 Byte)](#feldname-flags-1-byte)
    - [Detail-Informationen zum Status daten\[5\]](#detail-informationen-zum-status-daten5)
  - [Feldname: `data[5]` (5 \* 2Bytes)](#feldname-data5-5--2bytes)
    - [Aufbau bei CMD\_WRITE](#aufbau-bei-cmd_write)
    - [Aufbau bei DIGITAL\_WRITE / DIGITAL\_READ](#aufbau-bei-digital_write--digital_read)
      - [Neu in v2 (Digital\_WRITE/DIGITAL\_READ)](#neu-in-v2-digital_writedigital_read)
      - [Digital-GPIOs für die typischen ESP32-Derivate](#digital-gpios-für-die-typischen-esp32-derivate)
    - [Aufbau bei ANALOG\_WRITE / ANALOG\_READ](#aufbau-bei-analog_write--analog_read)
      - [Analog-GPIOs für die typischen ESP32-Derivate](#analog-gpios-für-die-typischen-esp32-derivate)
  - [Antwort-Paketstruktur (binäres Format, 17 Bytes Gesamt)](#antwort-paketstruktur-binäres-format-17bytes-gesamt)
    - [READ Commands](#read-commands)
  - [Feldname: `reserved` (1 Byte)](#feldname-reserved-1-byte)
  - [Feldname: CRC8 (1 Byte)](#feldname-crc8-1-byte)

## PIN-OUT ESP32 - RaspberryPI

|    ESP32    | R-PI | Info                                 |
| :---------: | :--: | ------------------------------------ |
|     5V      | 2/4  | 5V                                   |
| GPIO21(SDA) |  3   | GPIO2 (SDA), 4.7-10k Pullup gegen 5V |
| GPIO22(SCL) |  5   | GPIO3 (SCL), 4.7-10k Pullup gegen 5V |
|     GND     |  6   | GND                                  |
|             |      |                                      |

## 📦 I²C-Paketstruktur (binäres Format, 17 Bytes Gesamt)

| Byte-Offset | Größe (Byte) | Typ           | Feldname   | Beschreibung                                  |
| ----------- | ------------ | ------------- | ---------- | --------------------------------------------- |
| 0–1         | 2            | `uint16_t`    | `header`   | Paketstart-Kennung – immer `0xFEEF`           |
| 2           | 1            | `uint8_t`     | `cmd`      | Command-ID (z. B. `1 = WriteServo`)           |
| 3           | 1            | `uint8_t`     | `subcmd`   | Subbefehl (z. B. `1 = Speed`, `2 = Steering`) |
| 4           | 1            | `uint8_t`     | `flags`    | Optionale Flags, Debug, Erweiterung           |
| 5–14        | 10           | `uint16_t[5]` | `data[5]`  | 5 Datenwerte (z. B. Geschwindigkeit, Winkel)  |
| 15          | 1            | `uint8_t`     | `reserved` | Für spätere Verwendung (aktuell `0`)          |
| 16          | 1            | `uint8_t`     | `crc`      | CRC8-Prüfsumme (Dallas/Maxim)                 |

**Gesamtlänge:** `17 Bytes`

> Hinweis: Die Daten werden im Little-Endian-Format übertragen.

## Feldname: Header `header`(2 Bytes)
Beginn eines Datenpaketes, grundsätzlich `0xFEEF`

## Feldname: Commands `cmd` (1 Byte)

| ID  | Name            | Beschreibung                                   |
| --- | --------------- | ---------------------------------------------- |
| 0   | `NONE`          | Kein Befehl                                    |
| 1   | `SERVO_WRITE`   | Steuert Dynamixel-Servos (z. B. Winkel, Speed) |
| 2   | `SERVO_READ`    | Liest den Status eines Servos aus              |
| 3   | `DIGITAL_WRITE` | Setzt den Wert eines digitalen Pins (0/1)      |
| 4   | `DIGITAL_READ`  | Liest den Wert eines digitalen Pins (0/1)      |
| 5   | `ANALOG_WRITE`  | Setzt den Wert eines analogen Pins (0–1023)    |
| 6   | `ANALOG_READ`   | Liest den Wert eines analogen Pins (0–1023)    |
| 7   | `ESP32_STATE`   | Status des ESP32 abfragen, nur lesen           |
|     |                 |                                                |

## Feldname: `SubCommmads` (1 Byte)

| SubCommandID             | Wert | Beschreibung                     |
| ------------------------ | ---- | -------------------------------- |
| SCMD_NONE                | 0    | Kein Befehl                      |
| SCMD_SERVO_SPEED         | 1    | Setze Geschwindigkeit des Servos |
| SCMD_SERVO_POSITION      | 2    | Setze Zielposition des Servos    |
| SCMD_SERVO_TORQUE_ENABLE | 3    | Aktiviere/Deaktiviere Torque     |
| SCMD_SERVO_LED_ON        | 4    | Schalte LED am Servo ein/aus     |
| SUB_MAX                  | 5    | Platzhalter / maximale ID        |

## Feldname: `flags` (1 Byte)
Wird bei READ_COMMANDs genutzt um einen Status zurück zu liefern.
Detail-Informationen zum Flag in den Daten[0...4] Array

|  BIT  | Name        | Beschreibung                                      |
| :---: | ----------- | ------------------------------------------------- |
|   0   | GENERAL     | Allgemeines Status flag, Details siehe Daten[0-4] |
|   1   | I2C         | Status des I2C Buses                              |
|   2   | DYNA        | Status der DynamixelServos                        |
|   3   | Free        |                                                   |
|   4   | Free        |                                                   |
|   5   | Status Free | Nur Flag, keine Details bei Daten                 |
|   6   | Status Free | Nur Flag, keine Details bei Daten                 |
|   7   | Status Free | Nur Flag, keine Details bei Daten                 |
|       |             |                                                   |

### Detail-Informationen zum Status daten[5]
| Index | Name | Beschreibung |
| :---: | ---- | ------------ |
|       |      |              |
|       |      |              |



## Feldname: `data[5]` (5 * 2Bytes)
Das Datenfeld `data` wird genutzt um 5 Float-Zahlen als`uint16_t` darzustellen.
Die übergebenen `FLOAT`-Werte liegen in einem Bereich von -1.0 bis +1.0. Um in Python eine 
einfache Generierung eines Binär-Protokolls zu erstellen müssen negative Werte in einen Zweierkomplement-Bereich konvertiert werden.


Jeder Wert in `data_vals` wird mit dem gegebenen `factor` multipliziert und anschließend
per Modulo 65536 in den Wertebereich eines `uint16_t` (0..65535) überführt.

Dies ist besonders nützlich für Binärprotokolle, bei denen negative Werte über das
Zweierkomplement als unsigned übertragen werden müssen.

### Aufbau bei CMD_WRITE

| IDX | Name     | Beschreibung                                   |
| --- | -------- | ---------------------------------------------- |
| 0   | VELOCITY | Wertebereich für die Servogeschwindigkeit      |
| 1   | STEERING | Wertebereich für die Lenkbewergung (Curvature) |
| 2   | free     | 0                                              |
| 3   | free     | 0                                              |
| 4   | free     | 0                                              |
|     |          |                                                |

### Aufbau bei DIGITAL_WRITE / DIGITAL_READ
Es können zwei Pins gleichzeitig gesetzt werden.

| IDX | Name | Beschreibung      | Info                             |
| --- | ---- | ----------------- | -------------------------------- |
| 0   | PIN1 | Pinnummer (GPIOx) | wenn 0 wird dieser Pin ignoriert |
| 1   | PIN2 | Pinnummer (GPIOy) | wenn 0 wird dieser Pin ignoriert |
| 2   | VAL1 | 0 oder 1 für Pin1 | ~~~~                                 |
| 3   | VAL2 | 0 oder 1 für Pin2 |                                  |
| 4   | free | 0                 |                                  |
|     |      |                   |                                  |

#### Neu in v2 (Digital_WRITE/DIGITAL_READ)
Insgesamt können bis zu fünf Pins gleichzeitig gesetzt werden

| IDX | MSB | LSB | Name         | Beschreibung                  | Info                                   |
| --- | --- | --- | ------------ | ----------------------------- | -------------------------------------- |
| 0   | x   | -   | Pin0 (GPIOx) | im MSB-Byte erster Pin        | wenn 0, wird dieser Pin igoniert       |
| 0   | -   | x   | Pin1 (GPIOx) | im LSB-Byte zweiter Pin       | wenn 0, wird dieser Pin igoniert       |
| 1   | x   | -   | Pin2 (GPIOx) | im MSB-Byte dritter Pin       | wenn 0, wird dieser Pin igoniert       |
| 1   | -   | x   | Pin3 (GPIOx) | im LSB-Byte vierter Pin       | wenn 0, wird dieser Pin igoniert       |
| 2   | x   | -   | Value Pin0   | 0 oder 1                      | alles > 0 = 1                          |
| 2   | -   | x   | Value Pin1   | 0 oder 1                      | alles > 0 = 1                          |
| 3   | x   | -   | Value Pin2   | 0 oder 1                      | alles > 0 = 1                          |
| 3   | -   | x   | Value Pin3   | 0 oder 1                      | alles > 0 = 1                          |
| 4   | x   | -   | Pin4(GPIOx)  | im MSB-Byte fünfter Pin4      | wenn MSB = 0, wird dieser Pin ignoiert |
| 4   | -   | x   | Value Pin4   | Value im LSB-Byte fünfer Pin4 | LSB, alles > 0 = 1                     |
|     |     |     |              |                               |                                        |


#### Digital-GPIOs für die typischen ESP32-Derivate
| ESP            | GPIO            | Type (Default Nutzung)      | Wertebereich (digital) |
| -------------- | --------------- | --------------------------- | ---------------------- |
| ESP32-WROOM-32 | 0               | Strapping Pin, Boot         | LOW/HIGH (0V–3.3V)     |
|                | 1 (TX0)         | UART0 TX (Debug)            | LOW/HIGH               |
|                | 3 (RX0)         | UART0 RX (Debug)            | LOW/HIGH               |
|                | 2               | General Purpose, LED oft    | LOW/HIGH               |
|                | 4, 5            | General Purpose             | LOW/HIGH               |
|                | 12–15           | Strapping Pins (Achtung!)   | LOW/HIGH               |
|                | 16, 17          | General Purpose             | LOW/HIGH               |
|                | 18, 19, 21      | SPI, I2C, General Purpose   | LOW/HIGH               |
|                | 22, 23          | I2C, General Purpose        | LOW/HIGH               |
|                | 25–27           | DAC, ADC2                   | LOW/HIGH               |
|                | 32–33           | Touch, ADC1                 | LOW/HIGH               |
|                | 34–39           | Input only, ADC1            | LOW/HIGH (nur Input)   |
| ESP32-S2       | 0–46 (außer 20) | General Purpose, USB, Touch | LOW/HIGH               |
| ESP32-S3       | 0–46 (außer 20) | General Purpose, USB, Touch | LOW/HIGH               |
| ESP32-C3       | 0–10            | I2C, UART, SPI, GPIO        | LOW/HIGH               |
| ESP32-C6       | 0–21            | I2C, UART, SPI, PWM, GPIO   | LOW/HIGH               |
| ESP32-H2       | 0–21            | I2C, UART, Zigbee, GPIO     | LOW/HIGH               |

**Erklärung der Spalte “Type”:**
- Strapping Pins: beeinflussen Bootverhalten, sollten beim Start nicht auf HIGH gezogen werden.
- UART0 TX/RX: werden meist für Debug-Ausgabe (Serial Monitor) verwendet.
- Touch: kapazitive Touch-Eingänge (z. B. GPIO 32–39).
- DAC: GPIO 25 & 26 unterstützen digitalen Analogausgang.
- Input only: GPIOs 34–39 können nur als Eingang verwendet werden.
- USB: bei S2/S3 werden GPIO19 & 20 oft für USB verwendet (nicht für GPIO-Zwecke nutzen).


### Aufbau bei ANALOG_WRITE / ANALOG_READ 
Es können zwei AnalogPins gleichzeitig gesetzt werden. verarbeiten können. Der DAC ist ein 12-Bit Converter

| IDX | Name | Beschreibung      | Pins |
| --- | ---- | ----------------- | ---- |
| 0   | PIN1 | Pinnummer (GPIOx) |      |
| 1   | PIN2 | Pinnummer (GPIOy) |      |
| 2   | VAL1 | 0-4095 für Pin1   |      |
| 3   | VAL2 | 0-4095 für Pin2   |      |
| 4   | free | 0                 |      |
|     |      |                   |      |

#### Analog-GPIOs für die typischen ESP32-Derivate
| ESP            | GPIO                         | Wertebereich (ADC) |
| -------------- | ---------------------------- | ------------------ |
| ESP32-WROOM-32 | 32–39 (ADC1)                 | 0 – 4095 (12 Bit)  |
|                | 0, 2, 4, 12–15, 25–27 (ADC2) | 0 – 4095 (12 Bit)  |
| ESP32-WROVER   | Gleich wie WROOM-32          | 0 – 4095 (12 Bit)  |
| ESP32-S2       | 1–20                         | 0 – 4095 (13 Bit)  |
| ESP32-S3       | 1–20                         | 0 – 4095 (13 Bit)  |
| ESP32-C3       | 0, 1, 2, 3, 4                | 0 – 4095 (12 Bit)  |
| ESP32-C6       | 0, 1, 2, 3, 4                | 0 – 4095 (12 Bit)  |
| ESP32-H2       | 0, 1, 2, 3                   | 0 – 4095 (12 Bit)  |
|                |                              |                    |

**Beispiel:**
    input:  [-1.0, 0.5, 1.0, 0.75,0]
    output: [65536 - 100 = 65436, 50, 100, 75, 0 ]

**Hinweise:**
- ADC1 vs. ADC2 (ESP32 Classic): ADC2 funktioniert nicht, wenn Wi-Fi aktiv ist, da ADC2 mit dem WLAN-Subsystem geteilt wird.
- Wertebereich kann per Software auf z. B. 0–1 V, 0–3.3 V angepasst sein. Standardmäßig meist 0–3.3 V bei 12 oder 13 Bit Auflösung.
- ESP32-S2/S3 haben eine verbesserte ADC-Kalibrierung und eine native 13-Bit-ADC-Unterstützung.

## Antwort-Paketstruktur (binäres Format, 17 Bytes Gesamt)
Die Struktur die der ESP32 an den Host sendet entspricht exakt der Struktur wie beim Empfangen.

| Feldname   | Beschreibung                                  | Daten      |
| ---------- | --------------------------------------------- | ---------- |
| `header`   | Paketstart-Kennung – immer `0xEFFE`           | 0xEFFE     |
| `cmd`      | Command-ID (z. B. `1 = WriteServo`)           | siehe oben |
| `subcmd`   | Subbefehl (z. B. `1 = Speed`, `2 = Steering`) | not used   |
| `flags`    | Optionale Flags, Debug, Erweiterung           | siehe oben |
| `data[5]`  | 5 Datenwerte (z. B. Geschwindigkeit, Winkel)  | siehe oben |
| `reserved` | Für spätere Verwendung (aktuell `0`)          | not used   |
| `crc`      | CRC8-Prüfsumme (Dallas/Maxim)                 | CRC        |
|            |
### READ Commands
  
|  ID   | Name         | Beschreibung                       | Daten                |
| :---: | ------------ | ---------------------------------- | -------------------- |
|   0   | not used     | -                                  | -                    |
|   1   | not used     | -                                  | -                    |
|   2   | SERVO_READ   | Daten der/des DynaServos           | data[0..4]           |
|   3   | not used     | -                                  | -                    |
|   4   | DIGITAL_READ | Status des/der Pins                | data[0], data[1]     |
|   5   | not used     | -                                  | -                    |
|   6   | ANALOG_READ  | Status des/der Analog-Pins         | data[0], data[1]     |
|   7   | ESP32_STATE  | Allgemeiner Status (siehe `flags`) | `flags` & data[0..4] |
|       |              |                                    |                      |

         |                                               |            |


## Feldname: `reserved` (1 Byte)
Aktuell nicht genutzt

## Feldname: CRC8 (1 Byte)
1Byte CRC.
