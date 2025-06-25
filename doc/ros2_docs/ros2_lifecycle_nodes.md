# ROS 2: Lifecycle Nodes im Rover-Projekt

## 🌐 Vergleich: Normaler Node vs. Lifecycle Node in ROS 2

| Kategorie                     | Normaler Node                          | Lifecycle Node                                                                                       |
| ----------------------------- | -------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| **Anwendungsbereich**         | Allgemeine ROS 2 Anwendungen           | Anwendungen mit kontrollierbarem Zustandsmodell (z. B. Sensor- oder Aktuatorsteuerung)               |
| **Zustandsverwaltung**        | Keine expliziten Zustände              | Hat definierte Zustände (z. B. `unconfigured`, `inactive`, `active`, `finalized`)                    |
| **Lebenszyklus-Steuerung**    | Nicht vorhanden                        | Lifecycle-Management mit Transitionen (z. B. `configure()`, `activate()`, `deactivate()`)            |
| **Startverhalten**            | Startet direkt nach Initialisierung    | Beginnt im Zustand `unconfigured` und benötigt explizite Transitionen zur Aktivierung                |
| **Sichtbarkeit über Tools**   | Sichtbar wie jeder andere Node         | Lifecycle-spezifische Tools können den Zustand und Übergänge anzeigen und steuern (`ros2 lifecycle`) |
| **Anwendungslogik-Kontrolle** | Vollständig im Code implementiert      | Zustandsbasierte Implementierung – Logik hängt vom aktuellen Zustand ab                              |
| **Service-Interfaces**        | Standard-ROS2-Services und -Topics     | Zusätzlich: Lifecycle-Service-Interface zur Zustandskontrolle                                        |
| **Fehlerbehandlung**          | Muss selbst implementiert werden       | Fehler führen zu definierten Zustandsübergängen (z. B. zurück zu `unconfigured`)                     |
| **Code-Komplexität**          | Einfacher Aufbau                       | Höhere Komplexität durch Zustandsverwaltung                                                          |
| **Beispiele**                 | - Logging-Node<br>- Bewegungssteuerung | - Kamera-Node mit kontrollierter Aktivierung<br>- Sensor-Node mit Rekonfiguration                    |

---

## 🔧 Wann sollte man Lifecycle Nodes verwenden?

Lifecycle Nodes bieten Vorteile, wenn:

- Komponenten dynamisch (de)aktiviert werden sollen (z. B. bei Rekonfiguration).
- Ein definierter Ablauf von Initialisierung bis Stilllegung notwendig ist.
- Ressourcen nur bei Bedarf aktiviert werden sollen (z. B. Hardware-Treiber).
- Eine explizite Kontrolle über die Zustände der Komponenten wichtig ist.

---

## 🧠 Beispiel aus dem Rover-Projekt

| Node-Typ     | Empfehlung                                                                                  |
| ------------ | ------------------------------------------------------------------------------------------- |
| `LEDNode`    | **Normaler Node**, da LEDs meist direkt auf Ereignisse reagieren.                           |
| `CameraNode` | **Lifecycle Node**, falls die Kamera nur bei Bedarf aktiviert wird.                         |
| `i2c_node`   | **Lifecycle Node**, wenn bestimmte Geräte initialisiert und ggf. deaktiviert werden müssen. |

---

## 🔍 Analyse deiner `rover-full.launch.py`

### ✅ **Empfohlene Lifecycle-Nodes**

| Node                     | Empfehlung                  | Begründung                                                                                                            |
| ------------------------ | --------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `odom_node`              | **Bereits LifecycleNode** ✅ | ✔️ Gut umgesetzt: Zustand steuerbar, sinnvoll bei Zustandswechseln im Navigationsstack.                                |
| `sensor_node`            | **Empfohlen**               | 🔄 Einbindung von Lidar-Sensoren erfordert typischerweise Initialisierung, Aktivierung, Fehlerbehandlung etc.          |
| `vision_node`            | **Empfohlen**               | 👁️ Kameras oder Vision-Algorithmen können aktiviert/deaktiviert werden, z. B. für Stromsparmodus oder Rekonfiguration. |
| `driver_controller_node` | **Optional**                | ⚙️ Falls die Steuerung rekonstruiert oder sicher aktiviert/deaktiviert werden soll.                                    |
| `led_node`               | ❌ Nicht empfohlen           | 🟢 LEDs benötigen typischerweise keinen Zustand – sie reagieren sofort auf eingehende Nachrichten.                     |
| `navigation_node`        | ❌ Nicht empfohlen           | ⚠️ Ist oft ein Wrapper um Verhalten; keine separaten Zustände notwendig.                                               |
| `tf2_*` Nodes            | ❌ Nicht empfohlen           | 🧭 Sind statische Publisher – keine Aktivierung/Deaktivierung notwendig.                                               |

---

## 🛠️ Umstellung auf LifecycleNode: Schritte am Beispiel `sensor_node`

### 1. Launch-Anpassung

```python
from launch_ros.actions import LifecycleNode

sensor_node = LifecycleNode(
    package='rover',
    executable='sensor_node',
    name='sensor_node',
    output='screen',
    parameters=[{
        'lidar_topic': TextSubstitution(text=lidar_topic)
    }]
)
```

## Anpassungen am Beispiel sensor_node.py
```
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

class SensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_node')
        self.get_logger().info("SensorNode constructed.")

    def on_configure(self, state: State):
        self.get_logger().info("Configuring...")
        # Sensor initialisieren
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("Activating...")
        # Publisher aktivieren
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("Deactivating...")
        # Publisher deaktivieren
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("Cleaning up...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State):
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

```

## LifeCycle Manager installieren
```
lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_all',
    output='screen',
    parameters=[{
        'autostart': True,
        'node_names': ['odom_node', 'sensor_node', 'vision_node']
    }]
)
```

## Vollständiges Beispiel von vision_node.py
```
# rover/src/rover/vision_node.py

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from std_msgs.msg import String

class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__('vision_node')
        self.get_logger().info('VisionNode constructed.')

        # Platzhalter für Publisher/Subscriber
        self.image_subscriber = None
        self.result_publisher = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_configure()')

        # Parameter auslesen (z. B. topic name)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        # Publisher vorbereiten (noch nicht aktiv)
        self.result_publisher = self.create_publisher(String, 'vision/result', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_activate()')

        # Subscriber aktivieren
        self.image_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )

        # Publisher aktivieren
        self.result_publisher.on_activate()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_deactivate()')

        # Subscriber deaktivieren
        if self.image_subscriber is not None:
            self.destroy_subscription(self.image_subscriber)
            self.image_subscriber = None

        # Publisher deaktivieren
        self.result_publisher.on_deactivate()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_cleanup()')

        # Ressourcen bereinigen
        if self.result_publisher is not None:
            self.destroy_publisher(self.result_publisher)
            self.result_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[VisionNode] on_shutdown()')
        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg: Image):
        self.get_logger().debug('[VisionNode] Received image')
        # Bildverarbeitung hier (Dummy-Logik)
        result = String()
        result.data = 'Erkanntes Objekt: <n/a>'
        self.result_publisher.publish(result)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

## sensor_node.py komplettes Beispiel
```# rover/src/rover/sensor_node.py

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan

class SensorNode(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_node')
        self.get_logger().info('SensorNode constructed.')

        # Platzhalter für Subscriber und Publisher
        self.lidar_subscriber = None
        self.scan_publisher = None
        self.lidar_topic = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_configure()')

        # Parameter deklarieren und auslesen
        self.declare_parameter('lidar_topic', '/scan')
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value

        # Publisher vorbereiten
        self.scan_publisher = self.create_publisher(LaserScan, 'sensor/scan', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_activate()')

        # Subscriber aktivieren
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            10
        )

        self.scan_publisher.on_activate()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_deactivate()')

        if self.lidar_subscriber is not None:
            self.destroy_subscription(self.lidar_subscriber)
            self.lidar_subscriber = None

        self.scan_publisher.on_deactivate()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_cleanup()')

        if self.scan_publisher is not None:
            self.destroy_publisher(self.scan_publisher)
            self.scan_publisher = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('[SensorNode] on_shutdown()')
        return TransitionCallbackReturn.SUCCESS

    def lidar_callback(self, msg: LaserScan):
        # Hier könnte z. B. ein Preprocessing stattfinden
        self.get_logger().debug('[SensorNode] LaserScan empfangen')
        self.scan_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

## Marker Node für RViz
````
lifecycle_status_marker_node = Node(
    package='rover',
    executable='lifecycle_status_marker',
    name='lifecycle_status_marker',
    output='screen'
)
```