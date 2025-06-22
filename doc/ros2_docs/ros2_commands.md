# ROS2 Befehle und Parameter

| Befehl                        | Mögliche Parameter                                | Anwendungsgebiet                 | Info                                                                 | Beispiel |
|--------------------------------|--------------------------------------------------|----------------------------------|----------------------------------------------------------------------|---------|
| `ros2 run`                     | `<package_name> <executable_name>`               | Package ausführen                | Startet ein ausführbares ROS2-Programm aus einem installierten Package. | `ros2 run turtlesim turtlesim_node` |
| `ros2 launch`                  | `<package_name> <launch_file.py>`                | Launch-Dateien                   | Startet mehrere Nodes mit einer Launch-Datei.                        | `ros2 launch demo_nodes_cpp talker_listener.launch.py` |
| `ros2 topic list`              | `--verbose`                                      | Topics                           | Listet alle verfügbaren Topics auf.                                 | `ros2 topic list` |
| `ros2 topic echo`              | `<topic_name>`                                  | Topics                           | Zeigt die Nachrichten eines Topics in der Konsole an.               | `ros2 topic echo /cmd_vel` |
| `ros2 topic pub`               | `<topic_name> <msg_type> '{data}'`               | Topics                           | Sendet eine Testnachricht an ein Topic.                             | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'` |
| `ros2 topic info`              | `<topic_name>`                                  | Topics                           | Zeigt Infos über ein Topic (Publisher/Subscriber).                   | `ros2 topic info /cmd_vel` |
| `ros2 node list`               | `--spin-time`                                   | Nodes                            | Listet alle aktiven ROS2-Nodes.                                      | `ros2 node list` |
| `ros2 node info`               | `<node_name>`                                   | Nodes                            | Zeigt Details zu einem Node (Topics, Services, Aktionen).           | `ros2 node info /turtle1` |
| `ros2 service list`            | `--verbose`                                     | Services                         | Listet alle verfügbaren Services.                                   | `ros2 service list` |
| `ros2 service call`            | `<service_name> <srv_type> '{args}'`             | Services                         | Sendet eine Anfrage an einen Service.                              | `ros2 service call /spawn turtlesim/srv/Spawn '{x: 2.0, y: 3.0, theta: 0.0}'` |
| `ros2 service type`            | `<service_name>`                               | Services                         | Zeigt den Typ eines Services.                                       | `ros2 service type /spawn` |
| `ros2 action list`             | `--verbose`                                     | Actions                          | Listet alle verfügbaren Actions.                                    | `ros2 action list` |
| `ros2 action send_goal`        | `<action_name> <action_type> '{args}'`           | Actions                          | Sendet ein Ziel an eine Action.                                     | `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'` |
| `ros2 action info`             | `<action_name>`                                | Actions                          | Zeigt Infos zu einer Action.                                        | `ros2 action info /navigate_to_pose` |
| `ros2 param list`              | `<node_name>`                                   | Parameter                        | Listet alle Parameter eines Nodes.                                 | `ros2 param list /turtlesim` |
| `ros2 param get`               | `<node_name> <param_name>`                      | Parameter                        | Ruft den Wert eines Parameters ab.                                 | `ros2 param get /turtlesim background_r` |
| `ros2 param set`               | `<node_name> <param_name> <value>`              | Parameter                        | Ändert den Wert eines Parameters zur Laufzeit.                     | `ros2 param set /turtlesim background_r 150` |
| `ros2 bag record`              | `<topic1> [<topic2> ...] -o <bag_name>`         | Datenaufzeichnung                | Zeichnet Nachrichten eines oder mehrerer Topics auf.                | `ros2 bag record -o my_bag /cmd_vel /scan` |
| `ros2 bag play`                | `<bag_name>`                                   | Datenwiedergabe                   | Spielt gespeicherte ROS2-Daten wieder ab.                          | `ros2 bag play my_bag` |
| `ros2 launch`                  | `<package> <launch_file.py>`                    | Launch                           | Startet Nodes über eine Launch-Datei.                              | `ros2 launch my_package my_launch.py` |
| `colcon build`                 | `--symlink-install --packages-select <pkg>`     | Build-System                      | Baut das ROS2-Workspace mit Colcon.                                | `colcon build --symlink-install` |
| `colcon test`                  | `--packages-select <pkg>`                       | Tests                            | Führt Tests für ein ROS2-Package aus.                              | `colcon test --packages-select my_package` |
| `colcon clean`                 |                                                | Build-System                      | Löscht erstellte Build-Dateien.                                    | `colcon clean` |
| `rosdep install`               | `--from-paths src --ignore-src -r -y`           | Abhängigkeiten                   | Installiert alle benötigten Abhängigkeiten für ROS2.               | `rosdep install --from-paths src --ignore-src -r -y` |
| `ros2 pkg list`                | `--verbose`                                     | Packages                         | Listet alle installierten ROS2-Packages auf.                       | `ros2 pkg list` |
| `ros2 pkg create`              | `<package_name> --build-type ament_<type>`      | Packages                         | Erstellt ein neues ROS2-Package.                                   | `ros2 pkg create my_package --build-type ament_python` |
| `ros2 doctor`                  |                                                | Systemdiagnose                   | Führt eine Systemdiagnose aus, um ROS2-Probleme zu finden.         | `ros2 doctor` |


## Beispiele zu den vorherigen Befehlen
## 1. `ros2 run`
| Beispiel | Info |
|----------|------|
| `ros2 run turtlesim turtlesim_node` | Startet den `turtlesim_node` aus dem `turtlesim`-Package. |
| `ros2 run demo_nodes_cpp talker` | Startet den `talker`-Node aus dem `demo_nodes_cpp`-Package. |

## 2. `ros2 launch`
| Beispiel | Info |
|----------|------|
| `ros2 launch turtlesim multisim.launch.py` | Startet mehrere TurtleSim-Nodes über eine Launch-Datei. |
| `ros2 launch nav2_bringup navigation2.launch.py` | Startet das Navigation2-Framework. |

## 3. `ros2 topic list`
| Beispiel | Info |
|----------|------|
| `ros2 topic list` | Listet alle aktiven Topics auf. |
| `ros2 topic list --verbose` | Zeigt zusätzliche Infos zu den Topics an. |

## 4. `ros2 topic echo`
| Beispiel | Info |
|----------|------|
| `ros2 topic echo /cmd_vel` | Zeigt die Nachrichten des Topics `/cmd_vel`. |
| `ros2 topic echo /scan` | Gibt die Sensordaten des LiDAR-Sensors aus. |

## 5. `ros2 topic pub`
| Beispiel | Info |
|----------|------|
| `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'` | Sendet eine Steuerungsnachricht an ein Roboter-Topic. |
| `ros2 topic pub /light std_msgs/msg/Bool '{data: true}'` | Schaltet eine LED über ein Topic ein. |

## 6. `ros2 topic info`
| Beispiel | Info |
|----------|------|
| `ros2 topic info /cmd_vel` | Zeigt Infos über das Topic `/cmd_vel` (Publisher, Subscriber). |
| `ros2 topic info /scan` | Zeigt Infos über das LiDAR-Topic `/scan`. |

## 7. `ros2 node list`
| Beispiel | Info |
|----------|------|
| `ros2 node list` | Listet alle aktiven Nodes im ROS2-Netzwerk. |
| `ros2 node list --spin-time 5` | Aktualisiert die Liste für 5 Sekunden. |

## 8. `ros2 node info`
| Beispiel | Info |
|----------|------|
| `ros2 node info /turtle1` | Zeigt Infos über den Node `/turtle1` an. |
| `ros2 node info /lidar_sensor` | Zeigt Details über den Node `/lidar_sensor`. |

## 9. `ros2 service list`
| Beispiel | Info |
|----------|------|
| `ros2 service list` | Listet alle aktiven Services. |
| `ros2 service list --verbose` | Zeigt detaillierte Infos zu den Services. |

## 10. `ros2 service call`
| Beispiel | Info |
|----------|------|
| `ros2 service call /reset std_srvs/srv/Empty` | Setzt den TurtleSim zurück. |
| `ros2 service call /spawn turtlesim/srv/Spawn '{x: 2.0, y: 3.0, theta: 0.0}'` | Erstellt eine neue Turtle bei (2,3). |

## 11. `ros2 action list`
| Beispiel | Info |
|----------|------|
| `ros2 action list` | Zeigt alle verfügbaren Actions. |
| `ros2 action list --verbose` | Zeigt zusätzliche Infos zu den Actions. |

## 12. `ros2 action send_goal`
| Beispiel | Info |
|----------|------|
| `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'` | Sendet eine Navigationsaktion an einen Roboter. |
| `ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci '{order: 5}'` | Startet eine Fibonacci-Berechnung als Action. |

## 13. `ros2 param list`
| Beispiel | Info |
|----------|------|
| `ros2 param list /turtlesim` | Listet alle Parameter des `turtlesim`-Nodes. |
| `ros2 param list /nav2_controller` | Zeigt Parameter des Navigations-Nodes an. |

## 14. `ros2 param set`
| Beispiel | Info |
|----------|------|
| `ros2 param set /turtlesim background_r 255` | Setzt die Hintergrundfarbe auf rot. |
| `ros2 param set /robot_speed 2.5` | Ändert die Geschwindigkeit eines Roboters. |

## 15. `ros2 bag record`
| Beispiel | Info |
|----------|------|
| `ros2 bag record -o my_bag /cmd_vel /scan` | Zeichnet Daten der Topics `/cmd_vel` und `/scan` auf. |
| `ros2 bag record /camera/image_raw` | Zeichnet Bilddaten einer Kamera auf. |

## 16. `ros2 bag play`
| Beispiel | Info |
|----------|------|
| `ros2 bag play my_bag` | Spielt die aufgezeichneten Daten aus `my_bag` ab. |
| `ros2 bag play lidar_data` | Gibt LiDAR-Daten erneut aus. |

## 17. `colcon build`
| Beispiel | Info |
|----------|------|
| `colcon build --symlink-install` | Baut das gesamte ROS2-Workspace. |
| `colcon build --packages-select my_package` | Baut nur das `my_package`-Package. |

## 18. `colcon test`
| Beispiel | Info |
|----------|------|
| `colcon test` | Führt alle Tests im Workspace aus. |
| `colcon test --packages-select my_package` | Führt Tests nur für `my_package` aus. |

## 19. `colcon clean`
| Beispiel | Info |
|----------|------|
| `colcon clean` | Löscht den Build-Ordner. |
| `colcon clean --build` | Entfernt nur die Build-Daten. |

## 20. `rosdep install`
| Beispiel | Info |
|----------|------|
| `rosdep install --from-paths src --ignore-src -r -y` | Installiert alle Abhängigkeiten im `src`-Verzeichnis. |
| `rosdep install --rosdistro humble --ignore-src` | Installiert Abhängigkeiten für ROS2 Humble. |

## 21. `ros2 pkg list`
| Beispiel | Info |
|----------|------|
| `ros2 pkg list` | Listet alle installierten Packages. |
| `ros2 pkg list --verbose` | Zeigt detaillierte Infos zu den Packages. |

## 22. `ros2 pkg create`
| Beispiel | Info |
|----------|------|
| `ros2 pkg create my_package --build-type ament_python` | Erstellt ein neues ROS2-Package mit Python. |
| `ros2 pkg create my_cpp_pkg --build-type ament_cmake` | Erstellt ein neues ROS2-Package mit C++. |

## 23. `ros2 doctor`
| Beispiel | Info |
|----------|------|
| `ros2 doctor` | Diagnostiziert das ROS2-System. |
| `ros2 doctor --report` | Erstellt einen ausführlichen Diagnosebericht. |