# LEDNode Nutzung
LEDNode ist ein Subscriber-Node zur Darstellung von farbigen Pattern auf vier WS2812-Ringen.

# Subscription
LEDNode subscribel das Topic `/led`.
Der Aufbau der `LEDMessage` entspricht nachfolgenden Attributen:

```
int32 pattern
string ledtype
int32 timeout
int32 duration
int32 duration_on
int32 duration_off
float32 brightness
int32 ledmask

# Neue Felder
int32[3] color     # RGB: z.B. [255, 100, 0]
string callback    # Name des Musters oder Methode, z. B. "blink"
```

**LEDMessage:**
- **color** : array [r,g,b], setzt Farbmuster   
- **pattern** : beschreibt welches Muster angezeigt werden soll (siehe LEDPattern(Enum)
- **ledtype** : Default "WS2812"
- **timeout** : Angabe in Millisekunden (ms). Zeit gibt an, wie lange ein Pattern aktiv (ON) ist bevor es ausgeschaltet wird (OFF)
- **duration_on** : Angabe in Millisekunden wie lange die ON-Phase ist (bei Blinkpattern)
- **duration_off** : Angabe in Millisekunden, wie lange die OFF-Pahse ist (bei Blinkpattern)
- **brightness** : Default 0.3, helligkeit der LEDs. Maximaler Wert 1.0 - ACHTUNG bei 28LEDs fließen dann bei weiß fast rund 1.6A(60mA pro LED * 28 ~1600mA)
- **ledmask** : Binär Muster welche LEDs ON/OFF geschaltet werden sollen. Überschreibt das gewählte LEDPattern. Beschreibt 28Bits. Jedes Bit entspricht einer LED.
- **callback**: String mit einem der Befehle: `fill, blink, run, circle`
- 
# Rover-Pattern
Für alle Publisher steht eine LEDPatter ENUM zur Verfügung das genutzt werden kann. Der Publisher schreibt `LEDMessage ins Topic` und unmittelbar danach für `led_node` alle notwendigen Dinge durch.

`led_node` implementiert alle Patterns die in `LEDPattern` definiert sind.

## Pattern-Konfigurationen - Neues Pattern erstellen
Es können in diese YAML-Datei eigene Patterns konfiguriert werden. Wenn man ein eigenes (neues) erstellen möchte
Nachfolgend ein Beispiel:

```
68:
  <<: [*CENTER_ALL, *defaults]
  name: "BATTERY_30"
  color: [255, 76, 0]
```
- **68**: muss ein eindeutiger Key (integer) sein. Dieser Key muss auch in LEDPattern-Enum erfasst werden
- **<<: [Erbt von `*CENTER_ALL`, Erbt von `*defaults`] (Mehrere Parents)
  - **<<: `*defaults` wenn nur ein Parent verwendet wird
- **name**: Kurze Beschreibung des Patterns. Ändlicher Key sollte in LEDPattern verwendet werden
- **duration_on**: überschreibt aus den Parents
- **duration_off**: überschreibt aus den Parents
- **ledmask**: überschreibt den Wert aus den Parents
- **timeout**: überschreibt aus den Parents
- **color**: Array in From [r,g,b] überschreibt aus den Parents
- **callback**: nutzt diesen Callback, überschreibt den callback aus den Parents. Mögliche Callbacks `fill(), blink(), run(), circel()`

**WICHTIG**
Jeder Key aus der YAML-Datei muss auch als Key/Value in LEDPattern vorhanden sein

## LEDPattern
```
from enum import Enum

class LEDPattern(Enum):
    """
    Enumeration zur einfacheren Nutzung der unterschiedlichen Patterns
    """
    NONE=0
    OFF=1
    UNKNOWN_ERROR=10


    # fill()
    RED=50
    GREEN=51
    BLUE=52
    YELLOW=53
    PINK=54
    WHITE=55
    AQUA=56
    LILA=57
    GREENYELLOW=58
    GREENBLUE=59
    ORANGE=60

    # Battery-Level
    BATTERY_100=61
    BATTERY_90=62
    BATTERY_80=63
    BATTERY_70=64
    BATTERY_60=65
    BATTERY_50=66
    BATTERY_40=67
    BATTERY_30=68
    BATTERY_20=105
    BATTERY_10=104

    # blink()
    HAZARD = 100
    BLINK_LEFT = 101
    BLINK_RIGHT = 102
    
    # blink()
    BATTERY_0 = 103
```

# LED-Darstellung
Die LED-Darstellung ist festimplementierung und kann nicht vom Publisher geändert werden. Der Publisher kann lediglich das Pattern wählen und über die Parameter, Zeit, Farbe und LED-Muster (ledmask) anpassen.

Nachfolgende Routinen sind fest verankert im Treiber und werden durch `led_node` basierend auf `LEDPattern` definiert

## fill()
Ist das einfachste aller Darstellungsmöglichkeiten. LEDs werden einfach ON/OFF geschaltet. `timeout` steuert wie lange die LEDs leuchten. Ist `timeout` gleich 0, dann leuchten die LEDs solange bis sie explizit mit `OFF` abgeschaltet werden. Der Publisher muss diesen Befehl senden

## blink()
Läßt die in `ledmask` angegebenen LEDs blinken. `duration_on / duration_off` geben an wie lang die ON/OFF Phase ist. `timeout` steuert wie lange die LEDs leuchten. Ist `timeout` gleich 0, dann leuchten die LEDs solange bis sie explizit mit `OFF` abgeschaltet werden. Der Publisher muss diesen Befehl senden

## run()
Führt ein Muster durch, sequenziell nach einander für alle Ringe (1-4)

## circle()
Lauflicht für  alle LEDs in allen WS2812-Ringen (Ringe 1-4). Grundsätzlich ist LED1 (Bit0) nicht inbegriffen. 

# Bekannte Fehler
- run() und circle() haben noch keine wirklich optimale Implementierung.
- Wird ein unbekannter LEDPattern angegeben vom Publisher crashed led_node

# TEST eines Pattern
1.  Terminal1-Fenster: ros2 Rover Projekt starten
2.  Terminal2-Fenster: `ros2 topic echo /led` LED-Nachrichten überwachen
3.  Terminal3-Fenster: `ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 50, ledtype: 'WS2812', timeout: 2000, duration_on: 250, duration_off: 250, ledmask: 0b0001110001110011001, color: [100,200,250]}" --once`

## Nützliche Commands
- Zeigt den Aufbau einer LEDMessage
  - `ros2 interface show rover_interfaces/msg/LEDMessage` 
- Überwache /led-topic
  - `ros2 topic echo /led`
- LEDMessage publishen
  - `ros2 topic pub /led rover_interfaces/msg/LEDMessage "{...}"` --once
    - `--once` Message nur 1x senden. Lässt man den Parameter weg, werden laufend Messages versendet
- startet der LEDNode fehlerfrei
  - `ros2 run rover led_node
  - Es werden **keine** Patterns (YAML) geladen !!! Nur zum Testen ob der LEDNode fehlerfrei gestartet werden kann
```
 2 bernd@ros2pi5:~/ros2_ws$ ros2 run rover led_node
[INFO] [1752241764.702907941] [led_node]: 
        LEDNode config:
        --------------------------------
        Topic:          /led_default,
        LEDType:        WS2812,
        Pixels:         24,
        Brightness:     0.3,
        Timeout:        1000,
        DurationON:     500,
        DurationOFF:    500,
        
[INFO] [1752241764.703734718] [led_node]: [LEDPatternLoader] /home/bernd/ros2_ws/install/rover/share/rover/config/ledpatterns.yaml
[YAML Load] Überspringe Eintrag: 'anchors' (kein numerischer Pattern-Key)
[INFO] [1752241764.724046368] [led_node]: Verfügbare Pattern-IDs: [0, 1, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 100, 101, 102, 103, 104, 105]
[INFO] [1752241764.726488922] [led_node]: LEDNode gestartet
```

## Publish-LEDMessage Definition
Übergeben wird eine JSON-Struktur
```
{
  "pattern": 50,
  "ledtype": "WS2812",
  "timeout": 2000,
  "duration_on": 250,
  "duration_off": 250,
  "ledmask": "0b0010010001001011111101111110",
  "color": [
    100,
    200,
    250
  ]
}
```
- verwende Pattern: 50
- ledtype: default immer setzen
- timeout: 2000ms ON, dann OFF
- duration_on: 250ms (nur bei blink() wichtig)
- duration_off: 250ms (nur bei blink() wichtig)
- ledmask: überschreibt das in Pattern genutzte Pattern
- color : wenn Pattern <> 0 wird color ignoriert


```
{
  "pattern": 0,
  "ledtype": "WS2812",
  "timeout": 2000,
  "duration_on": 250,
  "duration_off": 250,
  "ledmask": "0b0001110001110011001",
  "color": [
    42,
    142,
    242
  ],
  "callback": "blink"
}
```
- ähnlich wie vorher
- **pattern**: Pattern=0 (NONE), Attribut `color` wird nun verwendet
- **callback**: leds sollen 2000ms blinken im Rythmus 250ms ON, 250ms OFF
- **color**: Farben: R=42, G=142, B=242