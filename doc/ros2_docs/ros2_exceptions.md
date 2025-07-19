# ROS2-Exceptions

| **Exception-Klasse**                    | **Beschreibung**                                                                                | **Typischer Kontext / Beispiel**                                   |
| --------------------------------------- | ----------------------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| `ROSInterruptException`                 | Wird bei Abbruch von `rclpy.spin()` durch Signale wie `SIGINT` (Ctrl+C) ausgelöst               | Node wird per `Ctrl+C` beendet                                     |
| `ParameterNotDeclaredException`         | Zugriff auf nicht deklarierten Parameter bei deaktivierter Option `allow_undeclared_parameters` | `node.get_parameter("foo")` ohne vorher `declare_parameter("foo")` |
| `InvalidParameterException`             | Allgemeiner Fehler beim Arbeiten mit Parametern                                                 | Ungültiger Typ oder Name bei `declare_parameter`                   |
| `InvalidParameterValueException`        | Parameterwert entspricht nicht den erlaubten Constraints (z. B. Range)                          | Bei Validierung mit Deskriptoren (Ranges, Typen)                   |
| `InvalidServiceNameException`           | Ungültiger Servicename (z. B. enthält Leerzeichen oder ungültige Zeichen)                       | `create_service(MySrv, "invalid name", callback)`                  |
| `InvalidTopicNameException`             | Ungültiger Topicname                                                                            | `create_publisher(Msg, "/invalid topic")`                          |
| `ServiceException`                      | Fehler beim Aufruf eines ROS-Services                                                           | `client.call(...)` oder `client.call_async(...)` schlägt fehl      |
| `TopicNameException`                    | Fehlerhafter Aufbau eines Topic-Namens (z. B. doppelte Slashes `//`)                            | Konstruktion eines falschen Namens per Parameter                   |
| `InvalidNodeNameException`              | Fehler beim Erstellen eines Nodes mit ungültigem Namen                                          | `Node("__bad_name__")`                                             |
| `NoExecutableFoundError` *(aus Launch)* | Beim Starten eines Launch-Files: Kein auszuführbares Programm gefunden                          | Launch-Datei verweist auf ungültiges Executable                    |
| `ClientNotAvailableException`           | Wenn ein Service-Client auf einen Server wartet, dieser aber nicht bereit ist                   | `client.wait_for_service(timeout_sec=2.0)` schlägt fehl            |
| `NodeNameNonExistentError`              | Zugriff auf einen nicht existenten Node (z. B. in Lifecycle oder über ROS-Tooling)              | z. B. bei `get_node_names()`                                       |
| `ParameterAlreadyDeclaredException`     | Wenn `declare_parameter()` mit einem bereits deklarierten Parameter aufgerufen wird             | Mehrfacher Aufruf mit gleichem Namen                               |
| `ParameterImmutableException`           | Wenn ein als "readonly" deklarierter Parameter zur Laufzeit geändert werden soll                | Nur beim Parameter-Descriptor `read_only=True`                     |