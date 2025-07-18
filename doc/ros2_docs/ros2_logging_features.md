
# ROS 2 Logging Levels in Python (rclpy)

ROS 2 verwendet ein Logging-System, das verschiedene Loglevels unterstützt. Diese lassen sich sowohl zur Laufzeit als auch zur Initialisierung eines Nodes anpassen.

---

## 🧭 Unterstützte Loglevels

Die Loglevels basieren auf `RCUTILS_LOG_SEVERITY`, entsprechend den Levels von `rclpy.logging`.

| Level        | Methode im Code    | Beschreibung                                  |
|--------------|--------------------|-----------------------------------------------|
| `DEBUG`      | `self.get_logger().debug()`   | Detaillierte Debug-Informationen (z.B. Variableninhalte) |
| `INFO`       | `self.get_logger().info()`    | Allgemeine Statusmeldungen                   |
| `WARN`       | `self.get_logger().warn()`    | Warnungen über potenzielle Probleme          |
| `ERROR`      | `self.get_logger().error()`   | Fehler, die aber nicht fatal sind            |
| `FATAL`      | `self.get_logger().fatal()`   | Schwere Fehler, die meist das Beenden erfordern |

---

## 🔧 Logging-Level zur Laufzeit setzen

### 1. **Direkt im Code**
```python
from rclpy.node import Node
import rclpy.logging

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info("Info log")
        self.get_logger().debug("Debug log")
```

---

### 2. **Per Parameter beim Starten**
ROS 2 unterstützt das Setzen des Log-Levels über das Argument `--ros-args`:

```bash
ros2 run my_package my_node_executable --ros-args --log-level DEBUG
```

Oder spezifisch pro Node (wenn mehrere Nodes im selben Prozess laufen):

```bash
ros2 run my_package my_node_executable \
  --ros-args --log-level my_node_name:=WARN
```

---

### 3. **In der Launch-Datei (Python)**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node_executable',
            name='my_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])
```

---

### 4. **Per YAML-Konfiguration**

Aktuell lässt sich der Loglevel **nicht direkt über die Standard-YAML-Parameterdatei** wie bei anderen Parametern setzen, da `log_level` kein normaler ROS-Parameter ist. Es gibt jedoch zwei Workarounds:

#### a) **YAML-basierte Konvention (Workaround im Code)**

```yaml
/**:
  ros__parameters:
    log_level: DEBUG
```

Dann im Node auslesen:

```python
level_str = self.get_parameter("log_level").get_parameter_value().string_value
from rclpy.logging import LoggingSeverity
log_level = getattr(LoggingSeverity, level_str.upper(), LoggingSeverity.INFO)
self.get_logger().set_level(log_level)
```

---

## 🧱 Empfehlungen (Best Practices)

| Anwendung | Empfehlung |
|----------|------------|
| Entwicklung | Setze `DEBUG` oder `INFO` direkt im Code oder per Launch-Argument |
| Produktion | Nutze `WARN` oder höher, um Logs übersichtlich zu halten |
| Launch-Dateien | Verwende `--ros-args --log-level`, um pro Node zu konfigurieren |
| YAML-Parameter | Nutze nur als Workaround – kein nativer Support für `log_level` |

---

## 📌 Zusammenfassung

| Methode              | Unterstützt `log_level`? | Geeignet für |
|----------------------|--------------------------|---------------|
| Python-Code direkt   | ✅                        | Laufzeit-Anpassung, dynamisch |
| `--ros-args`         | ✅                        | CLI-Start |
| Launch-Datei         | ✅                        | Automatisierter Start |
| YAML-Parameter       | ⚠️ (Workaround)           | Konfiguration über Parameterdateien |

---

## 🔗 Weiterführende Links

- [ROS 2 Logging Doku (rclpy)](https://docs.ros.org/en/rolling/How-To-Guides/Logging-and-logger-configuration.html)
- [rclpy Logging API](https://docs.ros2.org/latest/api/rclpy/rclpy.logging.html)
- [RCUTILS_LOG_SEVERITY](https://github.com/ros2/rcutils/blob/master/include/rcutils/logging.h)
