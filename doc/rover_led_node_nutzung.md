# LEDNode Nutzung
LEDNode ist ein Subscriber-Node zur Darstellung von farbigen Pattern auf vier WS2812-Ringen.

# Subscription
LEDNode subscribel das Topic `/led`.
Der Aufbau der `LEDMessage` entspricht nachfolgenden Attributen:

**LEDMessage:**
- **pattern** : beschreibt welches Muster angezeigt werden soll (siehe LEDPattern(Enum)
- **ledtype** : Default "WS2812"
- **timeout** : Angabe in Millisekunden (ms). Zeit gibt an, wie lange ein Pattern aktiv (ON) ist bevor es ausgeschaltet wird (OFF)
- **duration**: Angabe in Millisekdungen, Wert gilt dann sowohl für duration_on und duration_off. Duration: 500 heißt duration_on: 500, duration_off: 500
**aktuell nicht implementiert**
- **duration_on** : Angabe in Millisekunden wie lange die ON-Phase ist (bei Blinkpattern)
- **duration_off** : Angabe in Millisekunden, wie lange die OFF-Pahse ist (bei Blinkpattern)
- **brightness** : Default 0.3, helligkeit der LEDs. Maximaler Wert 1.0 - ACHTUNG bei 28LEDs fließen dann bei weiß fast rund 1.6A(60mA pro LED * 28 ~1600mA)
- **ledmask** : Binär Muster welche LEDs ON/OFF geschaltet werden sollen. Überschreibt das gewählte LEDPattern. Beschreibt 28Bits. Jedes Bit entspricht einer LED.

# Rover-Pattern
Für alle Publisher steht eine LEDPatter ENUM zur Verfügung das genutzt werden kann. Der Publisher schreibt `LEDMessage ins Topic` und unmittelbar danach für `led_node` alle notwendigen Dinge durch.

`led_node` implementiert alle Patterns die in `LEDPattern` definiert sind.


**Beachten**: aktuell sind noch nicht alle angegebenen Pattern implementiert.

## LEDPattern
```
class LEDPattern(Enum):
    """
    Enumeration zur einfacheren Nutzung der unterschiedlichen Patterns
    """
    # Full color
    RED=0
    GREEN=1
    BLUE=2
    ORANGE=3
    WHITE=4

    # Rover-Pattern
    BLINK_LEFT = 10
    BLINK_RIGHT = 11
    HAZARD_LIGHT = 12
    
    # Rover sensor_node
    BATTERY_100 = 100
    BATTERY_90 = 101
    BATTERY_80 = 102
    BATTERY_70 = 103
    BATTERY_60 = 104
    BATTERY_50 = 105
    BATTERY_40 = 106
    BATTERY_30 = 106
    BATTERY_20 = 107
    BATTERY_10 = 108
    BATTERY_LOW = 109

    # Rover - driver_controller_node
    DYNA_NOT_AVAILABLE = 200
    DYNA_PROBLEM = 201
    I2C_ESP32_NOT_FOUND = 250
    I2C_ESP32_STATE_ERR = 251

    # Rover - navigation_node
    # 300-399
    #     
    # Rover - vision_node
    # 400 - 499

    # Rover - odom_node
    # 500 - 599

    # Rover
    # Software
    PI_WIFI_NOT_AVAILABLE = 500
    ROVER_BOOT1 = 510
    ROVER_BOOT2 = 511
    ROVER_BOOT3 = 512
    ROVER_SHUTDOWN1 = 515
    ROVER_SHUTDOWN2 = 516


    UNKNOWN_ERROR = 1000
    OFF = 9999
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