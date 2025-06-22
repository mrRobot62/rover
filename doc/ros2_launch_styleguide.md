
# ✨ ROS 2 Launch File Styleguide

Ein konsistenter Stil in Launch-Dateien verbessert Wartbarkeit, Lesbarkeit und Zusammenarbeit im Team. Dieser Styleguide richtet sich an ROS 2-Projekte mit Python-basierten `.launch.py`-Dateien.

---

## 📁 Struktur der Datei

**Empfohlene Reihenfolge:**
1. **Imports**
2. **Globale Pfaddefinitionen / Hilfsfunktionen**
3. **Node-Deklarationen mit Kommentaren**
4. **LaunchDescription-Return mit logischer Reihenfolge**

---

## 📥 Imports

Verwende immer strukturierte Imports:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
```

---

## 📦 Pfade & Hilfsfunktionen

Pfadangaben sollten über `get_package_share_directory()` erfolgen und mit `os.path.join()` kombiniert werden:

```python
share_dir = get_package_share_directory('mein_paket')
config_file = os.path.join(share_dir, 'config', 'params.yaml')
```

---

## 🧱 Node-Deklaration (empfohlen: separat + kommentiert)

```python
# 📡 Sensor-Node – verarbeitet alle Sensordaten
sensor_node = Node(
    package='rover',
    executable='sensor_node',
    name='sensor_node',
    output='screen',
    parameters=[config_file]
)
```

Vermeide es, Nodes direkt im `return`-Block zu schreiben – Ausnahmen: ganz einfache Testdateien.

---

## 📝 Kommentare & Dokumentation

Verwende mehrzeilige Docstrings (`"""Kommentar"""`) für zusammengehörige Node-Blöcke  
und einfache `#`-Kommentare für Einzelzeilen.

---

## 🔧 Parameterdateien & Konfiguration

```python
DeclareLaunchArgument(
    'params_file',
    default_value=config_file,
    description='Pfad zur Parameterdatei'
)
```

Nutze Parameter immer über `LaunchConfiguration('params_file')` innerhalb von Nodes.

---

## 🧪 Bedingungen (Conditions)

Beispiel für bedingten Node-Start:

```python
Node(
    package='demo_pkg',
    executable='debug_node',
    condition=IfCondition(LaunchConfiguration('enable_debug'))
)
```

---

## 🌀 Lifecycle Nodes

Immer mit `LifecycleNode(...)` deklarieren. Lifecycle-Management idealerweise über `lifecycle_manager` starten.

```python
lifecycle_node = LifecycleNode(
    package='rover',
    executable='lifecycle_sensor',
    name='sensor_lifecycle',
    output='screen',
    parameters=[config_file]
)
```

---

## 📂 Andere Launch-Dateien einbinden

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('subsystem'), 'launch', 'subsystem_launch.py')
    ])
)
```

---

## 📋 Logging

```python
LogInfo(msg=['Starte Node: ', LaunchConfiguration('node_name')])
```

---

## 🧾 Rückgabe der LaunchDescription

```python
return LaunchDescription([
    DeclareLaunchArgument(...),
    LogInfo(...),
    sensor_node,
    driver_node,
    rviz_node
])
```

Reihenfolge: Erst Konfigurationen, dann Nodes, zuletzt Logging/Tools.

---

## ✅ Best Practices Checkliste

- [x] Strukturierte Imports
- [x] Kommentare bei jedem Node
- [x] Keine Hardcoded Paths
- [x] `LaunchConfiguration` für alles Parametrisierbare
- [x] Rückgabe am Ende sauber gegliedert
- [x] Lifecycle-Handling bei Hardware-Nodes
- [x] Logging zur Laufzeit verwenden

---

## 📚 Weiterführend

- [ROS 2 Launch-Tutorials (docs.ros.org)](https://docs.ros.org/en/rolling/How-To-Guides/Launch-system.html)
- [rclpy Lifecycle Node Tutorial](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Lifecycle-Nodes/Creating-A-Lifecycle-Node.html)
- [Best Practices Guide (community)](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/)

---
