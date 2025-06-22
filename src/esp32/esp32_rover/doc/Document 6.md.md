# Wichtige ROS 2 Terminal-Befehle

In diesem Dokument werden die wichtigsten ROS 2-Befehle vorgestellt, die im Terminal genutzt werden können. Zu jedem Befehl gibt es eine kurze Beschreibung und eine tabellarische Übersicht.

---

## ROS 2 Nodes
Nodes sind grundlegende Einheiten eines ROS 2-Systems. Sie kommunizieren miteinander über Topics, Services oder Actions. Die folgenden Befehle sind wichtig, um Nodes zu starten oder Informationen über sie abzurufen.

### Tabelle:
| Eigenschaft           | Beispiel                           | Information                                        |
|-----------------------|------------------------------------|--------------------------------------------------|
| Node starten          | `ros2 run <package_name> <node>`  | Startet einen Node aus einem bestimmten Package. |
| Liste aller Nodes     | `ros2 node list`                  | Zeigt eine Liste aller aktuell laufenden Nodes.  |
| Informationen zu Node | `ros2 node info <node_name>`      | Gibt Details zu einem spezifischen Node.         |


---

## ROS 2 Topics
Topics sind Kommunikationskanäle, über die Nodes Nachrichten austauschen. Diese Befehle helfen dabei, Topics zu überwachen, Informationen abzurufen oder Nachrichten zu senden.

### Tabelle:
| Eigenschaft               | Beispiel                          | Information                                      |
|---------------------------|-----------------------------------|------------------------------------------------|
| Liste aller Topics        | `ros2 topic list`                | Zeigt alle verfügbaren Topics an.              |
| Informationen zu Topic    | `ros2 topic info /<topic_name>`  | Zeigt Details zu einem bestimmten Topic.       |
| Nachrichten abonnieren    | `ros2 topic echo /<topic_name>`  | Gibt Nachrichten eines Topics in der Konsole aus. |
| Nachricht senden (publish)| `ros2 topic pub /<topic_name> <msg_type> <msg>` | Publiziert eine Nachricht auf ein Topic.       |

---

## ROS 2 Services
Services bieten synchronisierte Kommunikation zwischen Nodes. Sie werden genutzt, um Anfragen zu senden und Antworten zu erhalten.

### Tabelle:
| Eigenschaft              | Beispiel                            | Information                                      |
|--------------------------|-------------------------------------|------------------------------------------------|
| Liste aller Services     | `ros2 service list`                | Zeigt alle verfügbaren Services an.            |
| Informationen zu Service | `ros2 service info /<service_name>`| Zeigt Details zu einem bestimmten Service.     |
| Service aufrufen         | `ros2 service call /<service_name> <srv_type> <args>` | Ruft einen Service mit bestimmten Argumenten auf. |

---

## ROS 2 Actions
Actions werden für asynchrone Aufgaben genutzt, bei denen der Fortschritt überprüft werden soll. Diese Befehle helfen bei der Verwaltung von Actions.

### Tabelle:
| Eigenschaft               | Beispiel                              | Information                                      |
|---------------------------|---------------------------------------|------------------------------------------------|
| Liste aller Actions       | `ros2 action list`                   | Zeigt alle verfügbaren Actions an.             |
| Informationen zu Action   | `ros2 action info /<action_name>`    | Zeigt Details zu einer bestimmten Action.      |
| Action aufrufen (Send Goal)| `ros2 action send_goal /<action_name> <action_type> <args>` | Sendet ein Ziel (Goal) an eine Action.        |
---

## ROS 2 Parameters
Parameter sind wichtige Einstellungen für Nodes, die zur Laufzeit angepasst werden können. Die folgenden Befehle helfen, Parameter zu verwalten.

### Tabelle:
| Eigenschaft                | Beispiel                                 | Information                                      |
|----------------------------|------------------------------------------|------------------------------------------------|
| Liste aller Parameter      | `ros2 param list`                       | Zeigt alle Parameter eines Nodes an.           |
| Parameter auslesen         | `ros2 param get <node_name> <param_name>` | Gibt den Wert eines bestimmten Parameters aus. |
| Parameter setzen           | `ros2 param set <node_name> <param_name> <value>` | Setzt den Wert eines bestimmten Parameters.   |

---

## ROS 2 Launch Files
Launch Files dienen dazu, mehrere Nodes und Einstellungen gleichzeitig zu starten. Diese Befehle helfen, Launch-Files zu nutzen und zu testen.

### Tabelle:
| Eigenschaft             | Beispiel                             | Information                                    |
|-------------------------|--------------------------------------|----------------------------------------------|
| Launch File starten     | `ros2 launch <package_name> <file>` | Startet ein Launch-File aus einem Package.   |
| Launch File testen      | `ros2 launch --show-args`           | Zeigt mögliche Argumente für Launch-Files.   |

---

## ROS 2 Package Management
Packages sind Container für Nodes, Launch-Files und andere Ressourcen. Diese Befehle helfen beim Verwalten von ROS 2-Packages.

### Tabelle:
| Eigenschaft                   | Beispiel                            | Information                                    |
|-------------------------------|-------------------------------------|-----------------------------------------------|
| Liste aller Packages          | `ros2 pkg list`                    | Zeigt alle verfügbaren ROS 2 Packages an.     |
| Informationen zu Package      | `ros2 pkg info <package_name>`     | Gibt Details zu einem bestimmten Package.     |
| Package erstellen             | `ros2 pkg create <package_name>`   | Erstellt ein neues ROS 2 Package.            |

---

Damit solltest du die wichtigsten ROS 2-Befehle im Terminal verstehen und effektiv nutzen können. Falls du spezifische Beispiele oder tiefergehende Informationen zu einem der Befehle benötigst, lass es mich wissen! 🙂

