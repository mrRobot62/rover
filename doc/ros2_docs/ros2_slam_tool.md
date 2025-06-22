
# SLAM Toolbox – ROS 2 Dokumentation & Beispielkonfiguration

## 1. Allgemeine Beschreibung

`slam_toolbox` ist ein ROS 2-Paket für **Simultaneous Localization and Mapping (SLAM)**, das eine **2D-Kartenerstellung in Echtzeit oder im Offline-Modus** unterstützt. Es basiert auf einem optimierungsbasierten SLAM-Ansatz und bietet viele Funktionen für Mapping, Lokalisierung, Optimierung und Kartenspeicherung.

### Features:
- Echtzeitkartierung
- Offline-Kartierung mit Optimierung
- Pose-Graph-Optimierung
- Loop-Closure-Erkennung
- Speichern und Laden von Karten und Posen-Graphen
- Lebenslanges Mapping (Continuing Mapping)
- Visualisierung in RViz
- Kompatibel mit verschiedenen Lidar-Sensoren

---

## 2. Node-Varianten in `slam_toolbox`

| Node                        | Beschreibung                                     |
| --------------------------- | ------------------------------------------------ |
| `sync_slam_toolbox_node`    | Für Online-SLAM (synchronisiert mit Sensordaten) |
| `async_slam_toolbox_node`   | Für Online-SLAM (asynchron, mehr Flexibilität)   |
| `offline_slam_toolbox_node` | Für Offline-Mapping mit Daten aus ROS-Bags       |

---

## 3. Wichtige Parameter und Funktionen

| Parameter/Funktion                                | Anwendung                                                      | Beispiel                                        | Weitere Infos                                             |
| ------------------------------------------------- | -------------------------------------------------------------- | ----------------------------------------------- | --------------------------------------------------------- |
| `slam_toolbox` Nodes (`sync`, `async`, `offline`) | Auswahl des Betriebsmodus                                      | `ros2 run slam_toolbox async_slam_toolbox_node` | Async ist meist robuster bei Online-Mapping               |
| `/scan`                                           | Lidar-Topic, das vom SLAM verwendet wird                       | `/scan`                                         | Muss korrekt gesetzt sein (Lidar-Treiber)                 |
| `use_sim_time`                                    | Verwendung simulierter Zeit (z. B. bei Bagfiles)               | `true`                                          | Wichtig für Offline-Modus                                 |
| `odom_frame`                                      | Odometrie-Frame                                                | `"odom"`                                        | Muss zur TF-Hierarchie passen                             |
| `map_frame`                                       | Ziel-Frame der erzeugten Karte                                 | `"map"`                                         | Standard bei Navigation Stack                             |
| `base_frame`                                      | Roboter-Basisframe                                             | `"base_link"`                                   | Muss zur Roboter-TF-Struktur passen                       |
| `mode`                                            | Mapping-/Lokalisierungs-Modus                                  | `mapping` / `localization`                      | Kann zwischen Kartierung und Nur-Lokalisierung umschalten |
| `resolution`                                      | Kartenauflösung                                                | `0.05`                                          | Kleinere Werte = genauere Karte, aber größerer Speicher   |
| `max_laser_range`                                 | Maximal nutzbare Lidar-Reichweite                              | `8.0`                                           | Werte über Sensorreichweite ignorieren                    |
| `minimum_time_interval`                           | Zeitabstand zwischen zwei Messungen                            | `0.5`                                           | Verhindert Überlastung bei zu schneller Publikation       |
| `transform_publish_period`                        | Publikationsrate der map->odom Transformation                  | `0.05`                                          | Häufige Aktualisierung empfohlen                          |
| `map_update_interval`                             | Intervall zur Aktualisierung der Karte                         | `5.0`                                           | Kartenaktualisierung in Sekunden                          |
| `enable_interactive_mode`                         | Ermöglicht interaktives Hinzufügen/Entfernen von Nodes in RViz | `true`                                          | Nützlich für manuelle Optimierung                         |
| `load_state_filename`                             | Geladener Posen-Graph (JSON-Datei)                             | `my_map_posegraph.json`                         | Startet mit existierendem Mappingzustand                  |
| `serialization_format`                            | Speicherformat für Posen-Graph                                 | `json` / `cbor`                                 | JSON ist menschlich lesbar                                |
| `use_scan_matching`                               | Aktiviert Scan-Matching zur Verbesserung der Lokalisierung     | `true`                                          | Bessere Genauigkeit bei bewegtem Roboter                  |
| `use_rviz`                                        | RViz automatisch starten                                       | `true`                                          | Komfortfunktion beim Starten                              |
| `map_file_name`                                   | Dateiname für exportierte Karte                                | `map.yaml`                                      | Wird beim Speichern erzeugt                               |

---

## 4. Wichtige Services und Aktionen

| Service/Aktion                          | Anwendung                                 | Beispiel                                          | Weitere Infos                      |
| --------------------------------------- | ----------------------------------------- | ------------------------------------------------- | ---------------------------------- |
| `/slam_toolbox/save_map`                | Speichert aktuelle Karte (PGM + YAML)     | `ros2 service call /slam_toolbox/save_map`        | Kann jederzeit ausgelöst werden    |
| `/slam_toolbox/serialize_map`           | Speichert Posen-Graph in JSON/CBOR        | `ros2 service call /slam_toolbox/serialize_map`   | Graph zum späteren Laden           |
| `/slam_toolbox/clear`                   | Löscht Karte und Posen-Graph              | -                                                 | Startet Mapping neu                |
| `/slam_toolbox/deserialize_map`         | Lädt Posen-Graph aus Datei                | `ros2 service call /slam_toolbox/deserialize_map` | Ersetzt live Mapping-Daten         |
| `/slam_toolbox/loop_closure`            | Manuelle Loop-Closure auslösen            | -                                                 | Nur im interaktiven Modus sinnvoll |
| `/slam_toolbox/merge_maps`              | Verschmilzt zwei Karten                   | -                                                 | Selten benötigt, experimentell     |
| `/slam_toolbox/enable_interactive_mode` | Aktiviert interaktive Bearbeitung in RViz | -                                                 | Für manuelles Graph-Tuning         |

---

## 5. Beispielkonfiguration

### Projektstruktur:

```
my_robot_slam/
├── config/
│   └── slam_toolbox.yaml
├── launch/
│   └── slam_toolbox.launch.py
└── rviz/
    └── slam_toolbox.rviz
```

### slam_toolbox.yaml

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    mode: mapping
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_publish_period: 0.05
    map_update_interval: 5.0
    use_scan_matching: true
    use_scan_barycenter: false
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.1
    loop_closure_enabled: true
    loop_closure_frequency: 1.0
    enable_interactive_mode: true
    serialization_format: "json"
    load_state_filename: ""
    use_rviz: false
```

### slam_toolbox.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['../config/slam_toolbox.yaml']
        )
    ])
```

---

## 6. Karten speichern und laden

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'map'}"
```

Für Lokalisierung:

```yaml
mode: localization
load_state_filename: "/path/to/map_posegraph.json"
map_file_name: "/path/to/map.yaml"
```

---

## 7. Weiterführende Links

- GitHub: https://github.com/SteveMacenski/slam_toolbox
- Tutorials: https://navigation.ros.org/tutorials/docs/slam_toolbox.html
