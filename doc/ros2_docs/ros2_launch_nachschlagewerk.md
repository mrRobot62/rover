
# 📚 ROS 2 Launch-Dateien – Nachschlagewerk

## 🧾 Zweck
Launch-Dateien dienen dazu, mehrere Nodes, Parameter, Konfigurationen und Umgebungen **einheitlich und automatisiert zu starten**. Sie sind das Rückgrat komplexer ROS 2-Anwendungen.

---

## 📂 Dateiformat
ROS 2 verwendet primär **Python-basierte Launch-Dateien (`.py`)**. XML und YAML sind nur eingeschränkt nutzbar.

---

## 🧱 Grundstruktur

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mein_paket',
            executable='mein_node',
            name='mein_node_name',
            parameters=[{'param1': 'wert'}],
            remappings=[('/alt', '/neu')],
            output='screen'
        )
    ])
```

---

## 🔧 Wichtige Komponenten

### 🟩 `Node(...)`
Startet einen ROS 2 Node.

**Parameter:**
- `package`: Name des ROS 2 Pakets.
- `executable`: Name der ausführbaren Datei.
- `name`: (optional) Benennung des Nodes.
- `namespace`: (optional) ROS-Namespaces.
- `parameters`: Übergabe von Parametern (Datei oder Dict).
- `remappings`: Topic-Remapping.
- `output`: `screen` (Konsole) oder `log`.

---

### 📄 Parameter-Datei einbinden

```python
parameters=[PathJoinSubstitution([
    FindPackageShare('mein_paket'),
    'config',
    'params.yaml'
])]
```

---

### 📦 Launch anderer Dateien (`IncludeLaunchDescription`)

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('anderes_paket'),
            'launch',
            'andere_launch.py'
        ])
    ])
)
```

---

### 🌍 Umgebungsvariablen

```python
from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
```

---

### 🧪 Conditionals (optional starten)

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

Node(
    package='xyz',
    executable='abc',
    condition=IfCondition(LaunchConfiguration('use_feature'))
)
```

---

## 🛠️ Launch Configuration

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

DeclareLaunchArgument(
    'use_feature',
    default_value='false',
    description='Aktiviere spezielle Funktion'
)
```

---

## 🕹️ Lifecycle Node starten

```python
from launch_ros.actions import LifecycleNode

LifecycleNode(
    package='mein_paket',
    executable='mein_lifecycle_node',
    name='steuerung',
    namespace='roboter',
    output='screen',
    parameters=[{'use_sim_time': True}]
)
```

---

## 📣 Logging konfigurieren

```python
SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}] {message}')
```

---

## 🔄 Launch mit RViz und Nodes

```python
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_path],
    output='screen'
)
```

---

## 📎 Beispiel: Komplexe Launch-Datei mit Parametern & Bedingung

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='my_robot',
            executable='controller',
            name='ctrl',
            parameters=[{'speed': 1.0}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', '/home/user/rviz/config.rviz'],
            output='screen'
        )
    ])
```

---

## 🧭 Weiterführend

- [`launch.ros.org`](https://docs.ros.org/en/rolling/How-To-Guides/Launch-system.html)
- ROS 2 Demos: `ros2 launch demo_nodes_cpp talker_listener.launch.py`
- Lifecycle-Tutorial: `ros2 run lifecycle lifecycle_node_example`
