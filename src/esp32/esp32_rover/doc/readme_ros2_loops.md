# rclpy.spin vs. Benutzerdefinierte Schleife (while rclpy.ok())

## Vergleich von rclpy.spin(node) und einer benutzerdefinierten Schleife
- **rclpy.spin(node)**: Verwendet, wenn der Node nur ROS2-Events wie Nachrichten oder Timer verarbeitet.
- **while rclpy.ok()**: Verwendet, wenn zusätzliche Aufgaben wie TCP-Kommunikation erforderlich sind.

### Tabelle: Vergleich rclpy.spin(node) vs. while rclpy.ok()
| Eigenschaft                      | rclpy.spin(node)                                              | Benutzerdefinierte Schleife (while rclpy.ok())                 |
|----------------------------------|-------------------------------------------------------------|----------------------------------------------------------------|
| **Hauptzweck**                   | Verarbeitet Ereignisse automatisch (Nachrichten, Timer).    | Ermöglicht zusätzliche benutzerdefinierte Aufgaben.            |
| **Flexibilität**                 | Für einfache Subscriber/Publisher geeignet.                 | Ermöglicht komplexere Aufgaben wie TCP-Kommunikation.          |
| **Ereignisverarbeitung**         | Automatisch durch ROS2 gesteuert.                           | Du musst explizit ROS2-Callbacks verarbeiten.                  |
| **Zusätzliche Aufgaben**         | Nicht direkt möglich.                                        | Zusätzliche Aufgaben wie TCP-Datenhandling möglich.            |
| **Steuerung**                    | ROS2 übernimmt die Steuerung der Ereignisverarbeitung.      | Volle Kontrolle über die Prozesslogik.                        |
| **Beispiele**                    | Subscriber, Publisher.                                       | Datenstreaming, Sensordatenverarbeitung, etc.                  |
| **Beendigung des Nodes**         | Automatisch bei STRG+C.                                     | Kann mit zusätzlicher Logik ergänzt werden.                   |