# 📦 ROS 2 Launch-Datei – `rover_full.launch.py`

Diese Datei beschreibt eine vollständige ROS 2 Launch-Konfiguration für ein Rover-Projekt. Sie beinhaltet die Initialisierung von Lidar-Sensorik, TF-Frames, Gamepad-Steuerung, Odometrie, Navigation, Vision und RViz.

## 🧠 Inhalt
- Parameterladefunktion
- Dynamische Sensorinitialisierung
- Transform-Definitionen
- Gamepad- und Navigationssteuerung
- Lifecycle und RViz
- Bewertung und Erweiterungsideen

---

## 📥 Parameter laden aus YAML

```python
def load_rover_params(path: str, lidar_model:st
    with open(path,'r') as f:
        data = yaml.safe_load(f)
        return data.get(lidar_model, {}).get("ros__parameters", {})
```

Lädt Parameter für das gewählte Lidar-Modell aus einer YAML-Datei.

---

## 🧾 Launch-Argumente

```python
lidar_model_arg = DeclareLaunchArgument(
    'lidar_model',
    default_value='ydlidar',
    description="Auswahl des Lidar-Models: ydlidar oder xv11"
)
```

Bestimmt, welches Lidar-Modell verwendet wird (`ydlidar` oder `xv11`).

...

## 🧩 Gruppenbildung & Rückgabe

```python
return LaunchDescription([
    lidar_model_arg,
    rviz_load_arg,
    params_file_arg,
    OpaqueFunction(function=create_lidar_node),
    core_nodes,
    nav_vision_nodes,
    OpaqueFunction(function=create_nodes_from_arguments)
])
```

Startet alle definieren Nodes und Gruppen abhängig von Startargumenten.

---

## ✅ Bewertung

| Kategorie            | Einschätzung                         |
| -------------------- | ------------------------------------ |
| Modularität          | ✅ Sehr gut durch Gruppen und Wrapper |
| Erweiterbarkeit      | ✅ Unterstützt mehrere Sensoren       |
| Lesbarkeit           | ✅ Klar strukturiert, gut benannt     |
| Wiederverwendbarkeit | ✅ YAML-basierte Konfiguration        |

---

## 💡 Verbesserungsvorschläge

- Lifecycle Manager für `odom_node` aktivieren
- URDF mit `robot_state_publisher` laden
- `use_sim_time`-Schalter ergänzen
- Logging in Datei ergänzen
- Optionale Namespace-Unterstützung einbauen
- `robot_localization` für Odom+IMU/GPS

---

📘 Diese Launch-Datei bildet eine solide Grundlage für ein ROS 2-basiertes Rover-System.
