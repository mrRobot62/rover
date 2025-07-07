# 🧠 ROS 2 Architektur für zentralen `i2c_node`

## 📘 Hintergrund

In ROS 2 Systemen mit mehreren Komponenten, die über den I2C-Bus kommunizieren, kann es zu **Zugriffskonflikten** und **Busblockaden** kommen, wenn mehrere Nodes direkt auf den I2C-Bus zugreifen. Um das zu verhindern, wird ein zentraler ROS 2 Node – der `i2c_node` – eingeführt.

Dieser Node übernimmt alle physischen I2C-Kommunikationsvorgänge, während andere ROS-Nodes über Topics oder Services mit ihm kommunizieren.

---

## 🏗️ Architekturübersicht

```
[driver_controller_node] ─────┐
                              │
[sensor_node] ───────────────-┼──▶ [i2c_node] ──> /dev/i2c-1
                              │        │
[other_node] ────────────────-┘        ├── ADS1115
                                       └── ESP32
```

Der zentrale `i2c_node` arbeitet als **LifecycleNode**, um strukturierte Zustandsübergänge und kontrollierten Systemstart zu ermöglichen.

---

## 🎓 Ziele der Architektur

- ❌ **Keine konkurrierenden I2C-Zugriffe**
- ✅ Klare Trennung zwischen Hardware-Treiber und ROS-Logik
- ✅ Flexibler Zugriff über Messages (publish/subscribe) und Services (request/response)
- ✅ Erweiterbar für weitere Slaves (ADS1115, IMU, etc.)

---

## 🔹 Beteiligte ROS 2 Messages & Services

### ✏️ `I2CWrite.msg`

Verwendet, um Schreibbefehle an den `i2c_node` zu senden.
Dies sind Steuerbefehle für den ESP32 und wird vom `driver_controller_node` genutzt

```
string command       # z. B. "digital_write", "servo"
int32[] pins         # Ziel-Pins auf dem ESP32
int32[] states       # Zustände HIGH/LOW oder PWM
int32 cmd            # CommandID
int32 subcmd         # SubCommandID
float64[] data       # z. B. Geschwindigkeit, Lenkung
```

**Beispiel:**
```
command: CommandID.SERVO_WRITE
subcmd: SubCommandID.SCMD_SERVO_SPEED
data: [1.0, 0.5, 0, 0, 0]
```

---

### 💡 `I2CReadResult.msg`

Wird vom `i2c_node` als Publisher versendet, um Ergebnisse von I2C-Lesevorgängen zu verbreiten.
Aktuelle Verwendung des ADS1115 Sensors. Zukünftig auch weitere I2C-Sensorn, für die kein build-in Message besteht.

**ANTWORT-Struktur**
```
# Request für den ESP32 verwendet wird
string device
int32 command           # Command für den Slave
int32 subcommand        # ggf. SubCommand für den Slave (z.B. esp32)
---
# Response
float32 fvalues         # gefüllt je nach command/subcommand
int32[] ivalues         # gefüllt je nach command/subcommand
```

**Typischer Use-Case:**
- Streaming von Analogwerten
- Ereignisbasierte Sensorzustände

---

### 📢 `I2Cesp32ReadRequest.srv`

ROS 2 Service zur gezielten Abfrage an den ESP32 stellt und das Ergebnis (Response) versendet
Typisches Szenario: Abfrage von Servo-Daten über den ESP32


**Request:**
```
string device           # "esp32"
int32 command           # Command für den Slave
int32 subcommand        # ggf. SubCommand für den ```

**Response:**
```
float32 fvalues         # gefüllt je nach command/subcommand
int32[] ivalues         # gefüllt je nach command/subcommand```

**Typischer Use-Case:**
- gezielte Abfrage von Servo-Zuständen

### 📢 `I2CReadRequest.srv`

ROS 2 Service zur gezielten generische Abfrage eines beliebigen I2C-Slaves
Rückgabe struktur gilt für alle Slaves gleichermaßen

**Request:**
```
string device           # "ads1115"
```

**Response:**
```
float32 fvalues         # grundsätzliche Rückgabe sind Float-Werte
```

**Typischer Use-Case:**
- gezielte Abfrage von Servo-Zuständen



---

## 🛠️ Implementierungsschritte

1. **Erstellen des zentralen `i2c_node`** als LifecycleNode
2. **Definieren der Schnittstellen:**
   - `I2CWrite.msg`, `I2CReadResult.msg`, `I2Cesp32ReadRequest.srv`,`I2CReadRequest.srv`
3. **Kommunikationsstruktur:**
   - Andere Nodes senden über `/i2c/write`
   - `i2c_node` sendet Ergebnisse über `/i2c/result`
   - Services werden über `/i2c/read` bereitgestellt
4. **Erweiterung für Slave-Typen:**
   - z. B. `ADS1115`, `ESP32`, weitere ADCs

---

## 🔄 Vergleich: Message vs. Service

| Kriterium      | `I2CReadResult.msg`    | `I2CReadRequest.srv`       |
| -------------- | ---------------------- | -------------------------- |
| Typ            | Publisher-Message      | Service (Request/Response) |
| Aufrufart      | automatisch / zyklisch | gezielte Anfrage           |
| Richtung       | `i2c_node` → andere    | andere → `i2c_node`        |
| Anwendungsfall | Sensor-Streaming       | gezielte Werteabfrage      |

---

## 📅 Beispielhafte Kommunikationsflüsse

### ✉️ Steuerbefehl (Velocity + Lenkung)
```
[DriverControllerNode] → /i2c/write ("servo") → [i2c_node]
```

### 🎧 Regelmäßige Sensordaten
```
[i2c_node] → /i2c/result ("ads1115") → [LoggerNode]
```

### ❓ Einmalige Abfrage
```
[ClientNode] → /i2c/read (srv) → [i2c_node] → Antwort mit Messwert
```

---

## 🚀 Vorteile der Architektur

- ✅ Sichere, konfliktfreie I2C-Kommunikation
- ✅ Klare ROS 2 Trennung zwischen Steuerung und Hardware
- ✅ Kombinierbare Zugriffsmethoden (publish + service)
- ✅ Zentrale Logging- und Diagnosefähigkeit

---

## 📊 Weiterer Ausbau

- Integration weiterer I2C-Geräte
- Caching & Buffering im `i2c_node`
- QoS-Einstellungen für Publisher/Subscriber
- Lebenszyklusmanagement in Launch-Dateien

---

Für Details siehe die einzelnen ROS 2 Komponenten:
- `rover_interfaces` (enthält .msg/.srv)
- `rover`-Paket mit `i2c_node.py`
- Launch-Dateien z. B. `rover-full2.launch.py`