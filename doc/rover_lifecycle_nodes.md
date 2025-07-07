# Lifecycle-Nodes im Rover Projekt
Detaillierte Beschreibung was LifeCycle Nodes sind findet man unter ros2_lifecycle_nodes.md

# Summary
**ROS 2 Lifecycle Nodes** sind speziell strukturierte Nodes mit definierten Zuständen wie  
**`unconfigured`**, **`inactive`**, **`active`** und **`finalized`**.  
Sie ermöglichen eine **kontrollierte Initialisierung und Aktivierung** von Systemkomponenten.  
Dadurch lassen sich z. B. Sensoren oder Aktoren gezielt **konfigurieren, starten, stoppen und zurücksetzen**.  
Lifecycle Nodes fördern ein **deterministisches, robustes Systemverhalten** – besonders relevant für sicherheitskritische oder modulare Roboterarchitekturen.


# Test ob ein LifeCycle_node überhaupt fehlerfrei läuft
Es werden für den Test **zwei Terminal-Fenster** benötigt



## TEST - Start des Nodes und Transitionswechsel durchführen
Nachfolgender Test ist nur zur Prüfung gedacht. Im laufenden Roverprojekt werden die Status-Transitionen (Übergänge) automatisiert

1. **Terminal 1:**
Hier wird der Node gestartet. Hier am Beispiel des `i2c_node`
`ros2 run rover i2c_node`

**Ausgabe:**
```
bernd@ros2pi5:~/ros2_ws$ ros2 run rover i2c_node
[INFO] [1751523993.419494945] [i2c_node]: I2C LifecycleNode instantiated
[INFO] [1751523993.421607332] [i2c_node]: I2CBus (<smbus2.smbus2.SMBus object at 0xffff8c144800>): ' /dev/i2c-18. Slave: 18
[INFO] [1751523993.422152961] [i2c_node]: I2CBus (<smbus2.smbus2.SMBus object at 0xffff8c144800>): ' /dev/i2c-18. Slave: 18
[INFO] [1751523993.422661405] [i2c_node]: 
I2CNode config:
--------------------------------
Topic WRITE:        /i2c/write,
Topic READ:         /i2c/read

```
In der Ausgabe sieht man keine Fehlermeldung. Alle LifeCycleNodes sind so implementiert, das sie eine Konfiguration anzeigen. In der Regel gibt es Einstellungen pro node in der `rover.yaml` Datei.

2. **Terminal 2:**
Hier wird nun der Status des laufenden LifeCycle-Nodes verändert. Dadurch sollte sich in der Ausgabe in Termin1 auch etwas ändern

2.1 **Transition zu: configure**
**Eingabe im Terminal 2**
`ros2 lifecycle set /i2c_node configure`

**Ausgabe in Terminal 1:**
```
[INFO] [1751529597.091228525] [i2c_node]: I2CNode configured.
[INFO] [1751529597.096589578] [i2c_node]: [I2CNode] Konfigurierung erfolgreich abgeschlossen.
```

2.2 **Transition zu: active**
**Eingabe im Terminal 2**
`ros2 lifecycle set /i2c_node activate`

**Ausgabe in Terminal 1:**
```
[INFO] [1751529662.293040184] [i2c_node]: [I2CNode] on_activate()
[INFO] [1751529662.293577110] [i2c_node]: [I2CNode] Aktivierung erfolgreich abgeschlossen.  
```
2.2 **Transition zu: deactive**
**Eingabe im Terminal 2**
`ros2 lifecycle set /i2c_node deactivate`

**Ausgabe in Terminal 1:**
```
[INFO] [1751529705.776557148] [i2c_node]: [I2CNode] on_deactivate()
[INFO] [1751529705.777392221] [i2c_node]: [I2CNode] Deaktivierung erfolgreich abgeschlossen.
```

2.3 **Transition zu: shutdown**
**Eingabe im Terminal 2**
`ros2 lifecycle set /i2c_node shutdown`
 
 **Ausgabe in Terminal 1:**
```
[INFO] [1751529895.815580292] [i2c_node]: [I2CNode] on_shutdown
[INFO] [1751529895.816366847] [i2c_node]: [I2CNode] Shutdown erfolgreich abgeschlossen.
```






