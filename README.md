# ROS2 ROVER Project

Dies ist das erste ROS2-Robotik Projekt das wir durchführen. Daher sind sicherlich einige Implementierungen die durchgeführt werden nicht ROS2 optimiert, stellen aber eine funktionierende Grundlage für weitere
Arbeiten zur Verfügung

**Robot-Rover Projekt** basierend auf ROS2 auf einem Raspberry PI4 oder PI5.
Akuell arbeiten wir getrennt mit zwei Rover die beide zwar auf ROS2 basieren aber doch unterschiedlich konfiguriert sind

## Historie
| Version |   Datum    | Inhalt                                                                                                       |
| :-----: | :--------: | ------------------------------------------------------------------------------------------------------------ |
|   0.1   | 2025-06-22 | initialer upload, Rover fährt grundlegene ROS2 Struktur funktioniert, ESP32 FW ok, Gamepad F710 ok, LIDAR ok |
|         |            |                                                                                                              |


## ROS2-HUMBLE
Hardware: Raspberry PI4, 4GB RAM, SD-Card
- **OS:** Ubuntu 22.04
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

# Ziel des Projektes
Ziel des Projektes ist ein autonomer Rover Roboter zu bauen, basierend auf ROS2 und diverser Sensoren.
- Komplexere Ansteuerung der Antriebskette durch einen Allradantrieb und Allradlenkung
- ROS2 lernen und verstehen
- Auslagerung von Funktionalität auf einen ESP32 inkl. der Erstellung der Firmware und Kommunikation mit dem PI (Host)
- Visualisierung des Rovers über RVIZ
- LIDAR-Sensor für die Hindernis-Erkennung
- Kamera für Objekterkennung, Hindernis-Vermeidung
- Navigation-Planning
- SLAM
- Odometrie
- ...

# Aktuelle Herausforderungen & Schwierigkeiten
-  ~~ROS2 param.yaml File wird nicht richtig an die Nodes übergeben und somit können Parameter von Nodes nicht ausgelesen werden~~
- Bei Erststart, starten direkt die Antriebsräder :-O. Fehlerursache muss in der Firmware liegen oder im driver_controller_node.py der einen unsinnigen Befehl weiterleitet
- Genereller Umgang mit Fehlern nicht oder nur ungenügend implementiert. Robustheit gegen Fehler noch nicht verfügbar
- Konfiguration der Antriebseinheiten aktuell in der ESP32-Firmware fest hinterlegt, das hat den Nachteil, das wir grundsätzlich zwischen Rover 1 und Rover 2 zwei unterschiedliche FW kompilieren müssen. Idee: Konfiguration auslagern und die Grundkonfiguration z.B. einem ESP32 SPIFF File ablegen. 
- 

# Die ROVER Fahrzeuge

## Bildmaterial

## Antrieb und Lenkung
Der Rover hat eine Allrad-Lenkung und wird von vier Antriebseinheiten gesteuert. Jede Antriebseinheit besteht aus zwei Dynamixel-AX12+ Servos. Ein Servo für den Radantribe (Velocity), ein Servo für die Lenkung des Rades (Steering). Angesteuert werden alle Servos über ein Dynamixel-BUS-Protokol (1.0 nicht 2.0!). Die dazugehörige Firmware wurde in einen ESP32 implementiert und bietet eine I2C Schnittstelle für die Kommunikation mit des Raspberry PI. Zur Ansteuerung von Dynamixel-Servos bedarf es einer kleinen Elektronik die exaktes Timing und Flanken für das BUS-Protokoll bereitstellt und basiert auf eine SN74LS241 und einem LevelShifter von 3.3v auf 5v.Details siehe ESP32-Schaltplan bzw. unter doc/esp32 die dazugehörigen Dokumentationen


## Start des Rovers
- login per SSH auf den PI des Rovers
- Wechsel in den Folder `ros2_ws` 
- ROS2-Environment aktivieren : `source install/setup.bash`
- ROVER starten (ohne RVIZ): ` ros2 launch rover rover_full.launch.py params-file:=install/rover/share/rover/config/rover.yaml lidar_model:=ydlidar use_rviz:=false`
- ROVER starten (mit RVIZ): ` ros2 launch rover rover_full.launch.py params-file:=install/rover/share/rover/config/rover.yaml lidar_model:=ydlidar use_rviz:=true`



# Aktuelle Folderstruktur (Stand 2025-06-22)
```
.
├── README.md
├── config
│   ├── rover.yaml
│   ├── rover.yaml.old
│   ├── rviz
│   │   └── rover.rviz
│   └── urdf
├── doc
│   ├── class_diagram.md
│   ├── esp32
│   │   ├── firmware_overview.md
│   │   ├── i2c_protokoll_v1.md
│   │   └── i2c_protokoll_v2.md
│   ├── images
│   ├── komponenten_diagram.md
│   ├── odometrie_allgemein.md
│   ├── ros2_docs
│   │   ├── ros2_commands.md
│   │   ├── ros2_commands.pdf
│   │   ├── ros2_launch_example.md
│   │   ├── ros2_launch_example.pdf
│   │   ├── ros2_micro_ros_details.md
│   │   ├── ros2_micro_ros_details.pdf
│   │   ├── ros2_micro_ros_esp32.md
│   │   ├── ros2_micro_ros_esp32.pdf
│   │   ├── ros2_slam_tool.md
│   │   ├── ros2_themen_welt.md
│   │   └── ros2_themen_welt.pdf
│   ├── ros2_launch_nachschlagewerk.md
│   ├── ros2_launch_styleguide.md
│   ├── rover_architektur.md
│   ├── rover_architektur_.md
│   ├── rover_architektur_tmp.html
│   ├── rover_full.launch.md
│   └── rover_module_doc.md
├── launch
│   ├── rover.launch.py
│   ├── rover_full.launch copy.py
│   ├── rover_full.launch.py
│   ├── rover_full.launch_20250501.py
│   ├── rover_param.launch.py
│   └── rover_param2.launch.py
├── package.xml
├── resource
│   └── rover
├── scripts
│   ├── lifecycle_dashboard.py
│   └── lifecycle_status_marker.py
├── setup.cfg
├── setup.py
├── src
│   ├── esp32
│   │   ├── esp32_rover
│   │   │   ├── deprecated
│   │   │   │   ├── __main_jsonData.cpp__20250419
│   │   │   │   ├── __old_rover.h__
│   │   │   │   ├── __old_rover_utils.h__
│   │   │   │   ├── __test__main.cpp__
│   │   │   │   └── _main_20250302_wifiversion.cpp_
│   │   │   ├── doc
│   │   │   │   ├── Document 6.md.md
│   │   │   │   ├── Rover_ESP32_Firmware_Doku.md
│   │   │   │   ├── Rover_ESP32_Firmware_rover_utils.md
│   │   │   │   ├── readme_esp32.md
│   │   │   │   ├── readme_esp32.md.sb-8cb1a911-kIORDL
│   │   │   │   └── readme_ros2_loops.md
│   │   │   ├── esp32_rover.code-workspace
│   │   │   ├── include
│   │   │   │   ├── DynamicArray.h
│   │   │   │   ├── README
│   │   │   │   ├── i2c_helper.h
│   │   │   │   ├── led_info.h
│   │   │   │   ├── rover.h
│   │   │   │   └── rover_utils.h
│   │   │   ├── lib
│   │   │   │   └── README
│   │   │   ├── platformio.ini
│   │   │   ├── src
│   │   │   │   └── main.cpp
│   │   │   └── test
│   │   │       └── README
│   │   └── esp32_rover.zip
│   └── rover
│       ├── __depr_drive_controller_node.py__
│       ├── __init__.py
│       ├── __pycache__
│       │   ├── __init__.cpython-312.pyc
│       │   ├── drive_controller_node.cpython-312.pyc
│       │   ├── driver_controller_node.cpython-312.pyc
│       │   ├── led_node.cpython-312.pyc
│       │   ├── manual_control_node.cpython-312.pyc
│       │   ├── navigation_node.cpython-312.pyc
│       │   ├── odom_node.cpython-312.pyc
│       │   ├── sensor_node.cpython-312.pyc
│       │   └── vision_node.cpython-312.pyc
│       ├── control
│       │   ├── __init__.py
│       │   ├── __pycache__
│       │   │   ├── __init__.cpython-312.pyc
│       │   │   └── led_pattern.cpython-312.pyc
│       │   ├── kinematics_model.py
│       │   ├── led_pattern.py
│       │   └── ws2812.py
│       ├── driver_controller_node.py
│       ├── hardware
│       │   ├── __init__.py
│       │   ├── __pycache__
│       │   │   ├── __init__.cpython-312.pyc
│       │   │   ├── i2c_driver.cpython-312.pyc
│       │   │   ├── lidar_driver.cpython-312.pyc
│       │   │   ├── rover_driver.cpython-312.pyc
│       │   │   ├── servo_driver.cpython-312.pyc
│       │   │   ├── steering_driver.cpython-312.pyc
│       │   │   ├── velocity_driver.cpython-312.pyc
│       │   │   ├── xv11_neato.cpython-312.pyc
│       │   │   └── ydlidar_tminiplus.cpython-312.pyc
│       │   ├── i2c_driver.py
│       │   ├── lidar_driver.py
│       │   ├── rover_driver.py
│       │   ├── xv11_neato.py
│       │   └── ydlidar_tminiplus.py
│       ├── led_node.py
│       ├── navigation_node.py
│       ├── odom_node.py
│       ├── perception
│       │   ├── __init__.py
│       │   ├── image_processor.py
│       │   └── obstacle_detector.py
│       ├── rover_config.py
│       ├── sensor_node.py
│       ├── sensors
│       │   ├── __init__.py
│       │   ├── __pycache__
│       │   │   ├── __init__.cpython-312.pyc
│       │   │   ├── battery_sensor.cpython-312.pyc
│       │   │   ├── lidar_sensor.cpython-312.pyc
│       │   │   └── sensor_interface.cpython-312.pyc
│       │   ├── battery_sensor.py
│       │   ├── camera_sensor.py
│       │   ├── lidar_sensor.py
│       │   ├── sensor_interface.py
│       │   └── ultrasound_sensor.py
│       └── vision_node.py
├── src_rover.code-workspace
└── test
    ├── i2c_binary_test.py
    ├── i2c_check_pi5.py
    ├── i2c_json_test.py
    ├── i2c_simple_test.py
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```
