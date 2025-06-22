```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file für das Roverprojekt. 
    Startet automatisch:
    1. Das `teleop_twist_joy`-Package für die Steuerung per Gamepad.
    2. Den `rover_4wheels`-Node für die Ansteuerung der Motoren.
    """
    return LaunchDescription([
        # Start teleop_twist_joy node für die Joystick-Steuerung
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                'require_enable_button': False,
                'enable_button': 5,  # Beispiel: Button 5 aktiviert Steuerung
                'axis_linear': {'x': 1},
                'scale_linear': {'x': 0.5},
                'axis_angular': {'yaw': 0},
                'scale_angular': {'yaw': 0.5},
            }]
        ),
        
        # Start rover_4wheels node zur Steuerung des Rovers
        Node(
            package='rover_4wheels',
            executable='rover_control',
            name='rover_4wheels',
            output='screen',
            parameters=[{
                'wheel_separation': 0.4,  # Abstand zwischen den Rädern (m)
                'wheel_radius': 0.1,      # Radius der Räder (m)
                'max_speed': 1.5,         # Maximale Geschwindigkeit (m/s)
            }]
        )
    ])
```

### Beschreibung der `launch.py` Datei

Diese Datei startet zwei ROS2 Nodes:
1. **teleop_twist_joy**: Wandelt Gamepad-Inputs in `Twist`-Nachrichten um, die zur Steuerung des Rovers genutzt werden.
2. **rover_4wheels**: Verantwortlich für die physische Bewegung des Rovers basierend auf empfangenen `Twist`-Befehlen.

### Parameter-Erklärung:
- **teleop_twist_joy**:
  - `require_enable_button`: Falls `True`, muss eine Taste gedrückt gehalten werden, um Eingaben zu senden.
  - `enable_button`: Button-ID für die Aktivierung (hier `5`).
  - `axis_linear` & `scale_linear`: Steuerung der Vorwärtsbewegung.
  - `axis_angular` & `scale_angular`: Steuerung der Drehbewegung.

- **rover_4wheels**:
  - `wheel_separation`: Abstand der Räder, beeinflusst die Steuerungsdynamik.
  - `wheel_radius`: Setzt die Größe der Räder.
  - `max_speed`: Definiert die Höchstgeschwindigkeit des Rovers.

### Verwendung:
Diese Datei wird genutzt, um das Rover-Steuerungssystem über eine Launch-Datei automatisch zu starten. Ausgeführt wird sie mit:
```bash
ros2 launch rover_4wheels rover_launch.py
```
Dabei wird der `teleop_twist_joy`-Node für die Steuerung per Joystick und der `rover_4wheels`-Node für die Motorsteuerung parallel gestartet.


# Parameter des `teleop_twist_joy`-Nodes

Der **`teleop_twist_joy`**-Node ist ein ROS2-Node, der Eingaben von einem **Joystick/Gamepad** in **`Twist`-Nachrichten** (`geometry_msgs/msg/Twist`) umwandelt. Diese Nachrichten werden typischerweise zur Steuerung von Robotern verwendet.

Nachfolgend sind alle wichtigen **Parameter** dieses Nodes mit detaillierten Erklärungen aufgelistet:

---

## **Hauptparameter von `teleop_twist_joy`**

### **1. Aktivierung des Joysticks**
| **Parameter**           | **Typ** | **Beschreibung**                                                                                                                                   |
| ----------------------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `require_enable_button` | `bool`  | Falls `True`, muss eine Taste gedrückt werden, bevor der Joystick aktiv ist. Falls `False`, ist der Joystick immer aktiv.                          |
| `enable_button`         | `int`   | Die ID des Joystick-Buttons, der gedrückt werden muss, um die Steuerung zu aktivieren. Beispiel: `5` für den **RB-Button** eines Xbox-Controllers. |

---

### **2. Steuerung der linearen Bewegung**
| **Parameter**        | **Typ** | **Beschreibung**                                                                                                |
| -------------------- | ------- | --------------------------------------------------------------------------------------------------------------- |
| `axis_linear`        | `dict`  | Ein Dictionary mit den Joystick-Achsen, die für die **lineare Bewegung** genutzt werden (z. B. `x`, `y`, `z`).  |
| `scale_linear`       | `dict`  | Skalierungswerte für die Geschwindigkeit entlang der jeweiligen Achsen (z. B. maximale Geschwindigkeit in m/s). |
| `scale_linear_turbo` | `dict`  | Skalierungswerte für eine **höhere Geschwindigkeit** (z. B. bei gedrücktem Turbo-Button).                       |

✅ **Beispiel:**
```yaml
axis_linear:
  x: 1    # Steuerung der Vorwärts-/Rückwärtsbewegung mit Joystick-Achse 1
  y: 0    # Seitwärtsbewegung mit Achse 0
  z: 3    # Vertikale Bewegung mit Achse 3

scale_linear:
  x: 1.0  # Standardgeschwindigkeit in x-Richtung
  y: 1.0  # Standardgeschwindigkeit in y-Richtung
  z: 1.0  # Standardgeschwindigkeit in z-Richtung

scale_linear_turbo:
  x: 2.0  # Erhöhte Geschwindigkeit für Turbo-Modus
  y: 2.0
  z: 2.0
```
---

### **3. Steuerung der Drehbewegung**
| **Parameter**   | **Typ** | **Beschreibung**                                                                                                    |
| --------------- | ------- | ------------------------------------------------------------------------------------------------------------------- |
| `axis_angular`  | `dict`  | Ein Dictionary mit den Joystick-Achsen, die für die **Drehbewegung** genutzt werden (z. B. `yaw`, `pitch`, `roll`). |
| `scale_angular` | `dict`  | Skalierungswerte für die Winkelgeschwindigkeit der Drehbewegungen (z. B. maximale Drehgeschwindigkeit in rad/s).    |

✅ **Beispiel:**
```yaml
axis_angular:
  yaw: 2      # Joystick-Achse 2 für Drehung um die Hochachse
  pitch: 4    # Joystick-Achse 4 für Kippen nach vorne/hinten
  roll: 5     # Joystick-Achse 5 für seitliches Kippen

scale_angular:
  yaw: 1.0    # Standarddrehgeschwindigkeit in rad/s
  pitch: 1.0
  roll: 1.0
```
---

### **4. Tastensteuerung (Buttons)**
| **Parameter** | **Typ** | **Beschreibung**                                                                                            |
| ------------- | ------- | ----------------------------------------------------------------------------------------------------------- |
| `buttons`     | `dict`  | Definiert Buttons für verschiedene Steuerfunktionen, z. B. Turbo-Modus oder alternative Steuerungsmethoden. |

✅ **Beispiel:**
```yaml
buttons:
  enable_turbo: 7   # Button 7 (z. B. RT-Trigger) aktiviert Turbo-Modus
  enable_standard: 6 # Button 6 (z. B. LT-Trigger) deaktiviert Turbo-Modus
```
---

## **Zusammenfassung**
Die `teleop_twist_joy`-Node ermöglicht die **komplette Steuerung eines Roboters über ein Gamepad**, indem sie **Joystick-Achsen und Buttons** mit **Bewegungs- und Drehkommandos** verknüpft.

Die wichtigsten Funktionen:
- **Linearbewegung:** Über `axis_linear` und `scale_linear`
- **Drehbewegung:** Über `axis_angular` und `scale_angular`
- **Turbo-Modus:** Erhöhte Geschwindigkeit mit `scale_linear_turbo`
- **Button-Steuerung:** Zusätzliche Funktionen für Aktivierung und Moduswechsel

---

## **Start der Node mit eigener Konfiguration**
```bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file my_teleop_params.yaml
```
🔹 *Hierbei wird `my_teleop_params.yaml` als Parameterdatei geladen.*

---

## **Fazit**
Mit diesen Parametern kann **jede beliebige Joystick-Konfiguration** umgesetzt werden, um die Steuerung optimal an den gewünschten Roboter anzupassen. 🚀

# Parameter des `teleop_twist_joy`-Nodes

Der **`teleop_twist_joy`**-Node ist ein ROS2-Node, der Eingaben von einem **Joystick/Gamepad** in **`Twist`-Nachrichten** (`geometry_msgs/msg/Twist`) umwandelt. Diese Nachrichten werden typischerweise zur Steuerung von Robotern verwendet.

Nachfolgend sind alle wichtigen **Parameter** dieses Nodes mit detaillierten Erklärungen aufgelistet:

---

## **Hauptparameter von `teleop_twist_joy`**

### **1. Aktivierung des Joysticks**
| **Parameter**           | **Typ** | **Beschreibung**                                                                                                                                   |
| ----------------------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `require_enable_button` | `bool`  | Falls `True`, muss eine Taste gedrückt werden, bevor der Joystick aktiv ist. Falls `False`, ist der Joystick immer aktiv.                          |
| `enable_button`         | `int`   | Die ID des Joystick-Buttons, der gedrückt werden muss, um die Steuerung zu aktivieren. Beispiel: `5` für den **RB-Button** eines Xbox-Controllers. |

---

### **2. Steuerung der linearen Bewegung**
| **Parameter**        | **Typ** | **Beschreibung**                                                                                                |
| -------------------- | ------- | --------------------------------------------------------------------------------------------------------------- |
| `axis_linear`        | `dict`  | Ein Dictionary mit den Joystick-Achsen, die für die **lineare Bewegung** genutzt werden (z. B. `x`, `y`, `z`).  |
| `scale_linear`       | `dict`  | Skalierungswerte für die Geschwindigkeit entlang der jeweiligen Achsen (z. B. maximale Geschwindigkeit in m/s). |
| `scale_linear_turbo` | `dict`  | Skalierungswerte für eine **höhere Geschwindigkeit** (z. B. bei gedrücktem Turbo-Button).                       |

✅ **Beispiel:**
```yaml
axis_linear:
  x: 1    # Steuerung der Vorwärts-/Rückwärtsbewegung mit Joystick-Achse 1
  y: 0    # Seitwärtsbewegung mit Achse 0
  z: 3    # Vertikale Bewegung mit Achse 3

scale_linear:
  x: 1.0  # Standardgeschwindigkeit in x-Richtung
  y: 1.0  # Standardgeschwindigkeit in y-Richtung
  z: 1.0  # Standardgeschwindigkeit in z-Richtung

scale_linear_turbo:
  x: 2.0  # Erhöhte Geschwindigkeit für Turbo-Modus
  y: 2.0
  z: 2.0
```
---

### **3. Steuerung der Drehbewegung**
| **Parameter**   | **Typ** | **Beschreibung**                                                                                                    |
| --------------- | ------- | ------------------------------------------------------------------------------------------------------------------- |
| `axis_angular`  | `dict`  | Ein Dictionary mit den Joystick-Achsen, die für die **Drehbewegung** genutzt werden (z. B. `yaw`, `pitch`, `roll`). |
| `scale_angular` | `dict`  | Skalierungswerte für die Winkelgeschwindigkeit der Drehbewegungen (z. B. maximale Drehgeschwindigkeit in rad/s).    |

✅ **Beispiel:**
```yaml
axis_angular:
  yaw: 2      # Joystick-Achse 2 für Drehung um die Hochachse
  pitch: 4    # Joystick-Achse 4 für Kippen nach vorne/hinten
  roll: 5     # Joystick-Achse 5 für seitliches Kippen

scale_angular:
  yaw: 1.0    # Standarddrehgeschwindigkeit in rad/s
  pitch: 1.0
  roll: 1.0
```
---

### **4. Tastensteuerung (Buttons)**
| **Parameter** | **Typ** | **Beschreibung**                                                                                            |
| ------------- | ------- | ----------------------------------------------------------------------------------------------------------- |
| `buttons`     | `dict`  | Definiert Buttons für verschiedene Steuerfunktionen, z. B. Turbo-Modus oder alternative Steuerungsmethoden. |

✅ **Beispiel:**
```yaml
buttons:
  enable_turbo: 7   # Button 7 (z. B. RT-Trigger) aktiviert Turbo-Modus
  enable_standard: 6 # Button 6 (z. B. LT-Trigger) deaktiviert Turbo-Modus
```
---

## **Zusammenfassung**
Die `teleop_twist_joy`-Node ermöglicht die **komplette Steuerung eines Roboters über ein Gamepad**, indem sie **Joystick-Achsen und Buttons** mit **Bewegungs- und Drehkommandos** verknüpft.

Die wichtigsten Funktionen:
- **Linearbewegung:** Über `axis_linear` und `scale_linear`
- **Drehbewegung:** Über `axis_angular` und `scale_angular`
- **Turbo-Modus:** Erhöhte Geschwindigkeit mit `scale_linear_turbo`
- **Button-Steuerung:** Zusätzliche Funktionen für Aktivierung und Moduswechsel

---

## **Start der Node mit eigener Konfiguration**
```bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file my_teleop_params.yaml
```
🔹 *Hierbei wird `my_teleop_params.yaml` als Parameterdatei geladen.*

---

## **Fazit**
Mit diesen Parametern kann **jede beliebige Joystick-Konfiguration** umgesetzt werden, um die Steuerung optimal an den gewünschten Roboter anzupassen. 🚀



```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class Rover4Wheels(Node):
    """
    ROS2 Node für die Steuerung eines Rovers mit einer Allrad-Ackermann-Lenkung.
    
    Der Node empfängt `Twist`-Nachrichten über `/cmd_vel`, berechnet die entsprechenden
    Lenkwinkel und Radgeschwindigkeiten für alle vier Räder und veröffentlicht sie.
    
    Wichtige Eigenschaften:
    - Alle vier Räder können unabhängig voneinander lenken.
    - Die vorderen und hinteren Räder haben spiegelbildliche Ausschlagwinkel.
    - Die Geschwindigkeit der Räder variiert abhängig vom Wendekreisradius.
    
    Mathematische Grundlage:
    Die Lenkung erfolgt über Ackermann-Kinematik, wobei die Drehwinkel so berechnet werden,
    dass die Achsen der Räder auf einen gemeinsamen Wenderadius zeigen.
    
    Lenkwinkel-Berechnung:
    - Der Kurvenradius R hängt von der gewünschten Winkelgeschwindigkeit `angular_z` und der
      Fahrzeuggeschwindigkeit `linear_x` ab.
    - Die Winkel der einzelnen Räder werden so berechnet, dass sie zum gemeinsamen Wenderadius zeigen.
    - Die vorderen und hinteren Räder lenken spiegelbildlich: 
      - Wenn links vorne CW dreht, dreht sich hinten links CCW.
    
    Geschwindigkeit-Berechnung:
    - Die Geschwindigkeiten der Räder werden anhand des Abstands zum Wendekreismittelpunkt skaliert.
    - Räder auf der Kurveninnenseite drehen langsamer als Räder auf der Außenseite.
    """
    def __init__(self):
        super().__init__('rover_4wheels')
        
        # Deklaration von Parametern mit Standardwerten
        self.declare_parameter('track_width', 0.11)  # Spurweite in Metern (110 mm)
        self.declare_parameter('wheelbase', 0.15)  # Radstand in Metern (150 mm)
        self.declare_parameter('max_steering_angle', 240.0)  # Maximaler Lenkwinkel (Grad)
        self.declare_parameter('min_steering_angle', 60.0)  # Minimaler Lenkwinkel (Grad)
        self.declare_parameter('straight_steering_angle', 150.0)  # Geradeauswinkel (Grad)
        self.declare_parameter('max_speed', 1.5)  # Maximale Geschwindigkeit (m/s)
        
        # Abrufen der Parameterwerte
        self.track_width = self.get_parameter('track_width').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.straight_steering_angle = self.get_parameter('straight_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        
        # Subscriber für die `/cmd_vel`-Nachrichten
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher für die Radgeschwindigkeiten und Lenkwinkel
        self.front_left_steering_pub = self.create_publisher(Float32, '/front_left_steering_angle', 10)
        self.front_right_steering_pub = self.create_publisher(Float32, '/front_right_steering_angle', 10)
        self.rear_left_steering_pub = self.create_publisher(Float32, '/rear_left_steering_angle', 10)
        self.rear_right_steering_pub = self.create_publisher(Float32, '/rear_right_steering_angle', 10)
        
        self.front_left_wheel_pub = self.create_publisher(Float32, '/front_left_wheel_speed', 10)
        self.front_right_wheel_pub = self.create_publisher(Float32, '/front_right_wheel_speed', 10)
        self.rear_left_wheel_pub = self.create_publisher(Float32, '/rear_left_wheel_speed', 10)
        self.rear_right_wheel_pub = self.create_publisher(Float32, '/rear_right_wheel_speed', 10)
        
        self.get_logger().info(f'Rover4Wheels gestartet mit track_width={self.track_width}, wheelbase={self.wheelbase}, max_speed={self.max_speed}')
    
    def cmd_vel_callback(self, msg):
        """
        Callback-Funktion für `/cmd_vel`-Nachrichten.
        Berechnet die Lenkwinkel und Radgeschwindigkeiten basierend auf Allrad-Ackermann-Kinematik.
        """
        linear_x = msg.linear.x  # Vorwärts-/Rückwärtsbewegung
        angular_z = msg.angular.z  # Drehbewegung um die Hochachse
        
        if angular_z == 0:
            # Geradeausfahrt: Alle Räder bleiben auf der Mittelposition
            fl_angle = fr_angle = rl_angle = rr_angle = self.straight_steering_angle
        else:
            # Berechnung des Wendekreises
            turning_radius = self.wheelbase / math.tan(math.radians(angular_z * (self.max_steering_angle - self.min_steering_angle) / 2))
            
            # Berechnung der Lenkwinkel für alle vier Räder
            fl_angle = math.degrees(math.atan(self.wheelbase / (turning_radius - self.track_width / 2)))
            fr_angle = math.degrees(math.atan(self.wheelbase / (turning_radius + self.track_width / 2)))
            rl_angle = -fl_angle  # Spiegelbildlich
            rr_angle = -fr_angle  # Spiegelbildlich
        
        # Veröffentlichung der berechneten Lenkwinkel
        self.front_left_steering_pub.publish(Float32(data=fl_angle))
        self.front_right_steering_pub.publish(Float32(data=fr_angle))
        self.rear_left_steering_pub.publish(Float32(data=rl_angle))
        self.rear_right_steering_pub.publish(Float32(data=rr_angle))
        
        self.get_logger().info(f'Cmd_vel erhalten: linear_x={linear_x}, angular_z={angular_z}')
        self.get_logger().info(f'FL: {fl_angle}°, FR: {fr_angle}°, RL: {rl_angle}°, RR: {rr_angle}°')


def main(args=None):
    """
    Initialisiert den ROS2-Node und startet die Verarbeitung von Nachrichten.
    """
    rclpy.init(args=args)
    rover_node = Rover4Wheels()
    rclpy.spin(rover_node)
    rover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
