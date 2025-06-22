# ROS2 ROVER Project

Robot-Rover Projekt basierend auf ROS2 auf einem Raspberry PI4 oder PI5.
Akuell arbeiten wir getrennt mit zwei Rover die beide zwar auf ROS2 basieren aber doch unterschiedlich konfiguriert sind

## ROS2-HUMBLE
Hardware: Raspberry PI4, 4GB RAM, SD-Card
- **OS:** Ubuntu 20.04
- **ROS2:** Humble
- **LIDAR:** YDLidar TMini Pro
- **Antrieb:** 8x Dynamixel AX12+ Servos, 4 für Antrieb, 4 für Lenkung (Allradlenkung)
Steuerung:** Gampad Logitech F710
- **Autonom:** in Arbeit
- **Kamera:** in Arbeit

## ROS2-JAZZY
- **Hardware:** Raspberry PI5, 8GB RAM, SSD
- **OS:** Ubuntu 24.04
- **ROS2:** Jazzy
- **LIDAR:** YDLidar TMini Pro
- **Antrieb:** 8x Dynamixel AX12+ Servos, 4 für Antrieb, 4 für Lenkung (Allradlenkung)
- **Steuerung:** Gampad Logitech F710
- **Autonom:** in Arbeit
- **Kamera:** in Arbeit

## Antrieb und Lenkung
Der Rover hat eine Allrad-Lenkung und wird von vier Antriebseinheiten gesteuert. Jede Antriebseinheit besteht aus zwei Dynamixel-AX12+ Servos. Ein Servo für den Radantribe (Velocity), ein Servo für die Lenkung des Rades (Steering). Angesteuert werden alle Servos über ein Dynamixel-BUS-Protokol (1.0 nicht 2.0!). Die dazugehörige Firmware wurde in einen ESP32 implementiert und bietet eine I2C Schnittstelle für die Kommunikation mit des Raspberry PI. Zur Ansteuerung von Dynamixel-Servos bedarf es einer kleinen Elektronik die exaktes Timing und Flanken für das BUS-Protokoll bereitstellt und basiert auf eine SN74LS241 und einem LevelShifter von 3.3v auf 5v.Details siehe ESP32-Schaltplan bzw. unter doc/esp32 die dazugehörigen Dokumentationen

# Die ROVER Fahrzeuge


