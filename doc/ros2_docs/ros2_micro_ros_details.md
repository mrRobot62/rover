# Nutzung von micro-ROS zwischen einem Raspberry Pi 5 und einem ESP32

## 1. Einführung in micro-ROS

### Was ist micro-ROS?
micro-ROS ist eine für Mikrocontroller optimierte Version von ROS2 (Robot Operating System 2). Sie erweitert das ROS2-Ökosystem, indem sie es auf ressourcenbeschränkte eingebettete Systeme bringt. micro-ROS basiert auf **Micro XRCE-DDS**, einem kompakten DDS-Client, der für den Einsatz auf Geräten mit begrenztem Speicher und Rechenleistung entwickelt wurde.

micro-ROS bietet eine standardisierte Möglichkeit zur Kommunikation zwischen Mikrocontrollern und einem ROS2-System. Es ermöglicht Mikrocontrollern, ROS2-Nachrichten zu senden und zu empfangen, wodurch sie vollständig in ein ROS2-Ökosystem integriert werden können.

### Hauptmerkmale von micro-ROS
- **Optimiert für Mikrocontroller:** Kann auf Plattformen wie ESP32, STM32 oder Arduino ausgeführt werden.
- **Verwendet DDS für Kommunikation:** Nutzt das DDS-Protokoll, das auch in ROS2 verwendet wird, um eine nahtlose Integration zu gewährleisten.
- **Ermöglicht ROS2-Funktionalität auf Embedded-Systemen:** Unterstützung für Topics, Services, Parameter und Actions auf Mikrocontrollern.
- **Speichereffizient und Echtzeitfähig:** Läuft mit wenigen Kilobyte RAM und unterstützt Echtzeitkommunikation.

### Vergleich: ROS2 vs. micro-ROS
| Merkmal | ROS2 | micro-ROS |
|---------|------|-----------|
| Plattform | PCs, SBCs (z. B. Raspberry Pi) | Mikrocontroller (z. B. ESP32, STM32) |
| Kommunikation | DDS (Data Distribution Service) | Micro XRCE-DDS (kompakte DDS-Version) |
| Ressourcenverbrauch | Hoch (mehrere MB RAM) | Sehr niedrig (wenige KB RAM) |
| Unterstützte Features | Vollständige ROS2-Funktionalität | Begrenzte ROS2-Funktionalität für Embedded-Systeme |

### Architektur von micro-ROS
Ein typisches micro-ROS-Setup besteht aus drei Hauptkomponenten:
1. **micro-ROS-Agent (läuft auf Raspberry Pi 5):** Vermittelt die Kommunikation zwischen dem Mikrocontroller (ESP32) und dem ROS2-Ökosystem.
2. **micro-ROS-Client (läuft auf ESP32):** Führt ROS2-Knoten auf einem Mikrocontroller aus und kommuniziert mit dem Agenten über UART, WiFi oder Ethernet.
3. **ROS2-Host (z. B. Raspberry Pi 5 oder PC):** Führt reguläre ROS2-Knoten aus, die mit micro-ROS-Knoten interagieren.

---

## 2. micro-ROS Befehle

Hier eine Übersicht der wichtigsten micro-ROS-Befehle:

| Befehl | Beispiel | Anwendungsgebiet | Erläuterung |
|--------|---------|------------------|-------------|
| `micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200` | `micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200` | Startet den micro-ROS-Agenten über eine serielle Verbindung. | Verwendet, um micro-ROS über eine serielle Verbindung mit dem Host zu verbinden. |
| `ros2 topic pub` | `ros2 topic pub /led_toggle std_msgs/msg/Bool '{data: true}'` | Publish einer Nachricht | Sendet eine Nachricht von ROS2 an einen micro-ROS-Knoten. |
| `ros2 topic echo` | `ros2 topic echo /sensor_data` | Abonnement von micro-ROS Nachrichten | Zeigt die vom micro-ROS-Client gesendeten Nachrichten an. |
| `ros2 service call` | `ros2 service call /reset_motor std_srvs/srv/Empty` | Serviceaufruf | Ruft einen Service auf einem micro-ROS-Node auf. |
| `ros2 action send_goal` | `ros2 action send_goal /move_robot my_msgs/action/Move '{x: 5, y: 10}'` | Actionsteuerung | Sendet eine asynchrone Aktion an micro-ROS. |
| `ros2 param list` | `ros2 param list /esp32_node` | Parameterverwaltung | Listet alle verfügbaren Parameter eines micro-ROS-Nodes auf. |

---

## 3. Installation von micro-ROS auf Raspberry Pi 5 und ESP32

### 3.1 Installation auf dem Raspberry Pi 5

#### 1. **ROS2 auf dem Raspberry Pi 5 installieren**
Falls ROS2 noch nicht installiert ist, installiere **ROS2 Jazzy**:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-ros-base
```
Source das ROS2-Setup:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2. **micro-ROS-Agent installieren**
```bash
sudo apt install python3-pip
pip install micro_ros_agent
```
Starte den micro-ROS-Agenten:
```bash
micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

---

## Fazit
Mit diesem Setup kann ein **Raspberry Pi 5** ROS2-Commands an einen **ESP32** senden, der **Motoren und LEDs** steuert. Der Gamepad-Controller ermöglicht dabei die einfache Steuerung.

🚀 **ROS2 trifft Embedded-Systeme – Willkommen in der Zukunft der Robotik!**

