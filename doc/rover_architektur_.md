  
# 🧠 Rover-Klassenmodell & Architektur (ROS 2)
  
Dieses Dokument beschreibt das strukturierte Klassen- und Komponentensystem für einen ROS 2-basierten Allrad-Rover mit I2C-gebundener Hardwaresteuerung. Der Aufbau ist in zwei Hauptbereiche gegliedert: ROS 2 Nodes (Kommunikation, Steuerung) und fachlich-technische Klassen (Hardware, Sensorik, Verarbeitung).
  
## 📘 Inhaltsverzeichnis
  
- [🧠 Rover-Klassenmodell \& Architektur (ROS 2)](#-rover-klassenmodell--architektur-ros-2 )
  - [📘 Inhaltsverzeichnis](#-inhaltsverzeichnis )
- [🏗️ Architekturmodell Rover-Projekt (ROS 2) – 3-Schichten-Modell](#️-architekturmodell-rover-projekt-ros-2--3-schichten-modell )
  - [1. Node-Schicht (ROS 2 Communication Layer)](#1-node-schicht-ros-2-communication-layer )
    - [Aufgaben:](#aufgaben )
    - [Beispiele:](#beispiele )
  - [2. Control-Schicht (Logik / Steuerung / Verarbeitung)](#2-control-schicht-logik--steuerung--verarbeitung )
    - [Aufgaben:](#aufgaben-1 )
    - [Beispiele:](#beispiele-1 )
  - [3. Hardware-Schicht (Treiber / I2C Kommunikation)](#3-hardware-schicht-treiber--i2c-kommunikation )
    - [Aufgaben:](#aufgaben-2 )
    - [Beispiele:](#beispiele-2 )
  - [Visualisierung](#visualisierung )
  - [Vorteile dieser Architektur](#vorteile-dieser-architektur )
- [🧩 Komponentendiagramm](#-komponentendiagramm )
  - [Legende:](#legende )
  - [🧩 ROS2 Node - Schicht (ROS-spezifisch, verwenden `rclpy`)](#-ros2-node---schicht-ros-spezifisch-verwenden-rclpy )
    - [`SensorNode` (Lidar / Ultraschall / Kamera)](#sensornode-lidar--ultraschall--kamera )
      - [Publisher](#publisher )
      - [Subscriber](#subscriber )
    - [OdomNode](#odomnode )
      - [Publisher](#publisher-1 )
      - [Subscriber](#subscriber-1 )
    - [ManualControlNode (DriveControllerNode)](#manualcontrolnode-drivecontrollernode )
      - [Publisher](#publisher-2 )
      - [Subscriber](#subscriber-2 )
    - [DriveControllerNode](#drivecontrollernode )
    - [NavigationNode](#navigationnode )
  - [NavigationNode](#navigationnode-1 )
      - [Publisher](#publisher-3 )
      - [Subscriber](#subscriber-3 )
    - [VisionNode (zukünftig)](#visionnode-zukünftig )
      - [Publisher](#publisher-4 )
      - [Subscriber](#subscriber-4 )
  - [📦 1. Klassenmodell (strukturier- te Gliederung)](#-1-klassenmodell-strukturier--te-gliederung )
    - [MotorDriver](#motordriver )
    - [SteeringController](#steeringcontroller )
    - [KinematicsModel](#kinematicsmodel )
    - [SensorInterface](#sensorinterface )
    - [LidarSensor, UltrasoundSensor, CameraSensor](#lidarsensor-ultrasoundsensor-camerasensor )
    - [`ImageProcessor` (zukünftig)](#imageprocessor-zukünftig )
    - [`ObstacleDetector`](#obstacledetector )
    - [Hardwarenahe Klassen](#hardwarenahe-klassen )
  - [📎 Hinweise zur Erweiterbarkeit](#-hinweise-zur-erweiterbarkeit )
  
  
---
  
  
# 🏗️ Architekturmodell Rover-Projekt (ROS 2) – 3-Schichten-Modell
  
Dieses Dokument beschreibt die geplante Software-Architektur für dein ROS 2 Rover-Projekt.
Das Modell orientiert sich an einer klaren Schichtenarchitektur mit Trennung von Kommunikation, Fachlogik und Hardwarezugriff.
  
---
  
## 1. Node-Schicht (ROS 2 Communication Layer)
  
### Aufgaben:
- Implementierung aller ROS 2 Nodes
- Verarbeiten und Weiterreichen von ROS-Messages (Publisher, Subscriber, Actions, Services)
- Kein direkter Hardwarezugriff!
- Abhängig von `rclpy`, ROS-Interfaces und Topics
  
### Beispiele:
  
| Node                  | Aufgabe                                             | Typ                 |
| --------------------- | --------------------------------------------------- | ------------------- |
| `manual_control_node` | Empfängt `/cmd_vel` und delegiert an Controller     | Subscriber          |
| `sensor_node`         | Publiziert Sensordaten (LiDAR, Ultraschall, Kamera) | Publisher           |
| `odom_node`           | Berechnet und publiziert Odometrie                  | Publisher           |
| `navigation_node`     | Verwaltet SLAM, Navigation und Pathfinding          | Action-Client       |
| `vision_node`         | Führt Bildverarbeitung aus                          | Publisher / Service |
  
---
  
## 2. Control-Schicht (Logik / Steuerung / Verarbeitung)
  
### Aufgaben:
- Fachlogik: Berechnungen, Steuerstrategien
- Hardware-unabhängig
- Transformation von Steuerwerten in Zielgrößen für Aktoren
- Implementierung von Kinematik, Odometrie, Bildverarbeitung
  
### Beispiele:
  
| Klasse             | Aufgabe                                         |
| ------------------ | ----------------------------------------------- |
| `KinematicsModel`  | Umrechnung Twist <-> Radgeschwindigkeiten       |
| `ObstacleDetector` | Fusion von Sensorwerten zu Hindernisinformation |
| `ImageProcessor`   | Bildverarbeitung (z.B. Objekterkennung)         |
  
---
  
## 3. Hardware-Schicht (Treiber / I2C Kommunikation)
  
### Aufgaben:
- Abstraktion und Zugriff auf Hardware über definierte Schnittstellen
- Nutzung von I2C, UART, SPI oder direkter Bibliotheken
- Versenden fertiger Steuerwerte an Motorcontroller, Servos, Sensoren
  
### Beispiele:
  
| Klasse               | Aufgabe                                    |
| -------------------- | ------------------------------------------ |
| `ServoDriver`        | Gemeinsame I2C-Basisklasse                 |
| `VelocityController` | Steuerung der Antriebseinheiten (via I2C)  |
| `SteeringController` | Steuerung der Lenkservos (via I2C)         |
| `LidarSensor`        | Rohdatenempfang und Verarbeitung vom LiDAR |
| `UltrasoundSensor`   | Triggern und Messen per Ultraschall        |
  
---
  
## Visualisierung
  
```text
+---------------------------+
|      ROS 2 Node-Schicht   |  <-- rclpy, ROS2 Messages
+---------------------------+
| Control-Schicht           |  <-- Fachlogik, Verarbeitung
+---------------------------+
| Hardware-Schicht          |  <-- I2C / Treiber / Sensor-Access
+---------------------------+
```
  
---
  
## Vorteile dieser Architektur
  
| Vorteil                           | Nutzen                                                       |
| --------------------------------- | ------------------------------------------------------------ |
| Trennung von ROS 2 Abhängigkeiten | Austauschbar, testbar, isoliert                              |
| Klare Verantwortlichkeiten        | Nodes = Kommunikation / Control = Logik / Hardware = Zugriff |
| Zukünftig erweiterbar             | Sensoren, Motoren, weitere Aktoren einfach ergänzbar         |
| Perfekt für Unit-Tests            | Control- und Hardwareklassen einzeln testbar                 |
  
---
  
Erstellt für das Allrad-gelenkte ROS 2 Rover-Projekt mit modularer Schichtenarchitektur.
  
  
  
# 🧩 Komponentendiagramm
  

```
Error: mermaid CLI is required to be installed.
Check https://github.com/mermaid-js/mermaid-cli for more information.

Error: Command failed: npx -p @mermaid-js/mermaid-cli mmdc --theme default --input /tmp/crossnote-mermaid2025310-145991-1fe14gb.8b2o.mmd --output /home/bernd/ros2_ws/assets/4d75b18a7f49618cc1e3b90b89e804070.png
/bin/sh: 1: npx: not found

```  

  
  
## Legende:
- ROS2 Nodes = Kommunikationsebene
- Hardware = I2C Controller
- Sensors = Physische Sensoren
- Control = Fachlogik & Verarbeitung
- Topics = ROS 2 Kommunikation (Publisher / Subscriber)
- I2C = Steuerung Richtung ESP32 (Velocity / Steering)
- PUB = published Topic
- SUB = subscribe Topic
---
  
  
## 🧩 ROS2 Node - Schicht (ROS-spezifisch, verwenden `rclpy`)
  
### `SensorNode` (Lidar / Ultraschall / Kamera)
- **Verantwortungsbereich**
  - Zentrale Erfassung aller Sensordaten des Rovers.
  - Initialisiert und betreibt alle physischen Sensoren.
  - Publiziert Rohdaten in ROS Topics.
  
#### Publisher
  
| Topic               | Msg Type                   | Beschreibung          |
| ------------------- | -------------------------- | --------------------- |
| `/scan`             | `sensor_msgs/LaserScan`    | LiDAR Scan-Daten      |
| `/ultrasound`       | `sensor_msgs/Range`        | Ultraschall Messwerte |
| `/camera/image_raw` | `sensor_msgs/Image`        | Kamerabild            |
| `/imu/data`         | `sensor_msgs/Imu`          | IMU Bewegungsdaten    |
| `/battery_state`    | `sensor_msgs/BatteryState` | Akkustatus            |
  
#### Subscriber
  
| Topic | Msg Type | Beschreibung                              |
| ----- | -------- | ----------------------------------------- |
| —     | —        | SensorNode empfängt keine externen Topics |
  
---
  
### OdomNode
- **Verantwortung**: 
  - Berechnet und veröffentlicht die Odometrie des Roboters.
  - Nutzt Encoderwerte und evtl. IMU-Daten zur Positionsbestimmung.
  - Dient als Basis für die Navigation.
  
#### Publisher
  
| Topic   | Msg Type            | Beschreibung         |
| ------- | ------------------- | -------------------- |
| `/odom` | `nav_msgs/Odometry` | Berechnete Odometrie |
  
#### Subscriber
  
| Topic                  | Msg Type               | Beschreibung              |
| ---------------------- | ---------------------- | ------------------------- |
| `/imu/data`            | `sensor_msgs/Imu`      | IMU Daten für Odometrie   |
| Encoder Daten über I2C | intern oder Custom Msg | Motor-Daten zur Odometrie |
  
---
  
### ManualControlNode (DriveControllerNode)
- **Verantwortung**: 
  - Entgegennahme von Fahrbefehlen (`/cmd_vel`).
  - Umsetzung von Geschwindigkeit und Lenkkrümmung.
  - Ansteuerung des MotorDriver und SteeringController via I2C.
  
#### Publisher
  
| Topic           | Msg Type               | Beschreibung         |
| --------------- | ---------------------- | -------------------- |
| `/status/drive` | Custom oder `std_msgs` | Aktueller Fahrstatus |
  
#### Subscriber
  
| Topic      | Msg Type              | Beschreibung                     |
| ---------- | --------------------- | -------------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Fahrbefehle vom Gamepad / Teleop |
  
  
---
  
### DriveControllerNode
- **Verantwortung**: 
  
  
- **Verantwortung**: Empfang von Velocity-Commands, Umsetzung in Servopositionen und Motorwerte.
- **TOPIC**:
	- `/cmd_vel` (geometry_msgs/Twist)
	- `/status/drive` (z. B. Feedback über Ist-Werte)
  
### NavigationNode
- **Verantwortung**: Steuerung der SLAM- und Navigationsprozesse.
- **Verantwortung**: 
  
## NavigationNode
- **Verantwortung**: 
  - Globale Pfadplanung und Navigation zum Ziel.
  - Lokale Kollisionsvermeidung.
  - Nutzung von SLAM und Kartendaten.
- 
#### Publisher
  
| Topic      | Msg Type              | Beschreibung                                       |
| ---------- | --------------------- | -------------------------------------------------- |
| `/plan`    | `nav_msgs/Path`       | Geplanter Pfad                                     |
| `/cmd_vel` | `geometry_msgs/Twist` | Generierte Bewegungsbefehle an DriveControllerNode |
  
#### Subscriber
  
| Topic               | Msg Type                | Beschreibung                     |
| ------------------- | ----------------------- | -------------------------------- |
| `/odom`             | `nav_msgs/Odometry`     | Odometrie-Daten                  |
| `/scan`             | `sensor_msgs/LaserScan` | LiDAR-Umgebungsdaten             |
| `/ultrasound`       | `sensor_msgs/Range`     | Zusatz Hindernisinformationen    |
| `/vision/obstacles` | Custom Msg              | Hindernisse aus Bildverarbeitung |
  
  
### VisionNode (zukünftig)
**Verantwortung**: Bildverarbeitung (Obstacle Detection, Objekterkennung)
- **Verantwortung**: 
  - Bildverarbeitung zur Objekterkennung und Hindernisvermeidung.
  - Fusion von Bilddaten mit anderen Sensorquellen.
  - Publikation erkannter Hindernisse.
  
#### Publisher
  
| Topic               | Msg Type   | Beschreibung                                        |
| ------------------- | ---------- | --------------------------------------------------- |
| `/vision/obstacles` | Custom Msg | Hindernisse / erkannte Objekte aus Bildverarbeitung |
  
#### Subscriber
  
| Topic               | Msg Type            | Beschreibung        |
| ------------------- | ------------------- | ------------------- |
| `/camera/image_raw` | `sensor_msgs/Image` | Rohdaten der Kamera |
  
## 📦 1. Klassenmodell (strukturier- te Gliederung)
  

```
Error: mermaid CLI is required to be installed.
Check https://github.com/mermaid-js/mermaid-cli for more information.

Error: Command failed: npx -p @mermaid-js/mermaid-cli mmdc --theme default --input /tmp/crossnote-mermaid2025310-145991-149eh9s.jr09.mmd --output /home/bernd/ros2_ws/assets/4d75b18a7f49618cc1e3b90b89e804071.png
/bin/sh: 1: npx: not found

```  

---
  
  
⸻
  
### MotorDriver
- Hardwaresteuerung, z. B. I2C/UART-Kommunikation mit Motorcontroller
  
### SteeringController
- Umrechnung von Krümmung in Servo-RAW-Werte
- Unterstützt Allradlenkung (inkl. max/min Winkel etc.)
  
### KinematicsModel
- Berechnung der Fahrkinematik
- Umkehrkinematik für Odometrie
  
### SensorInterface
- Abstrakte Basisklasse für Sensoren (Lidar, Ultraschall, Kamera)
  
### LidarSensor, UltrasoundSensor, CameraSensor
- Spezifische Implementierungen
- Parsen von Rohdaten, ggf. Filterung
  
### `ImageProcessor` (zukünftig)
- Führt Bildverarbeitung durch, z. B.:
  - Objekterkennung (ML)
  - Linienverfolgung
  - Kantenerkennung
  
### `ObstacleDetector`
- Fusioniert Sensordaten von LiDAR, Kamera und Ultraschall
- Erkennt Hindernisse und gibt Zonen oder Objektlisten aus
  
---
### Hardwarenahe Klassen
- `ServoDriver`: Basisklasse für alle I2C-basierten Hardwaremodule
- `VelocityController`: Ableitung von `ServoDriver`, für Antrieb
- `SteeringController`: Ableitung von `ServoDriver`, für Lenkung
- Diese Klassen verwenden JSON über I2C zur Kommunikation mit dem ESP32
  
---
  
## 📎 Hinweise zur Erweiterbarkeit
  
- Die Architektur erlaubt einfache Erweiterung durch neue Sensoren oder Steuerkomponenten.
- Neue Nodes (z. B. Missionssteuerung, Autonomes Fahren) können nahtlos integriert werden.
- Eine Trennung von ROS-spezifischem Code und technischer Steuerlogik erhöht die Testbarkeit und Wartbarkeit.
  
---
  
Erstellt für das Projekt eines Allrad-gelenkten ROS 2 Rovers mit modularer Hardwarearchitektur.
  