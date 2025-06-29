# Rover-LED WS2812

## Installation
`pip install adafruit-blinka rpi_ws281x --break-system-packages`

In /boot/firmware/config.txt:
`dtoverlay=pio`

Für ältere Pis: (PI4, 3, ...)
In /boot/config.txt
`dtoverlay=pwm-2chan`

Anschließend `sudo reboot`


# led_node
Dieser Node ist dafür verantwortlich die LED-Ringe (4 Stüch á 7 LEDs) geziehlt anzusteuern. Die Reihenfolge der LED-Ringe ist wie folgt
- Ring 1 = Links hinten (LH)
- Ring 2 = Links vorne (LV)
- Ring 3 = Rechts vorne (RV)
- Ringe 4 = Rechts hinten (RH)

Ein Ring selbst besteht aus 7 WS2812 LEDS im Kreis angeordnet (6 LEDs) und eine LED mittig.
LED0 ist die in der Mitte
LED1 auf 9Uhr
LED2 auf 10Uhr
LED3 auf 14Uhr
LED4 auf 15Uhr
LED5 auf 16Uhr
lED6 auf 20Uhr

## LEDMessage
besteht aus
- pattern : beschreibt welches Muster angezeigt werden soll (siehe LEDPattern(Enum)
- ledtype : Default "WS2812"
- timeout : Angabe in Millisekunden (ms). Zeit gibt an, wie lange ein Pattern aktiv (ON) ist bevor es ausgeschaltet wird (OFF)
- duration: Angabe in Millisekdungen, Wert gilt dann sowohl für duration_on udn duration_off. Duration: 500 heißt duration_on: 500, duration_off: 500
- duration_on : Angabe in Millisekunden wie lange die ON-Phase ist (bei Blinkpattern)
- duration_off : Angabe in Millisekunden, wie lange die OFF-Pahse ist (bei Blinkpattern)
- brightness : Default 0.3, helligkeit der LEDs. Maximaler Wert 1.0 - ACHTUNG bei 28LEDs fließen dann bei weiß fast rund 1.6A(60mA pro LED * 28 ~1600mA)
- ledmask : Binär Muster welche LEDs ON/OFF geschaltet werden sollen. Überschreibt das gewählte LEDPattern. Beschreibt 28Bits. Jedes Bit entspricht einer LED.
```
            Ring4   Ring3   Ring2   Ring1
Beispiel: 0b0000000 0000000 100110 100110
LV & LH jeweils LED2+3+5
```
|  BIT  |  Ring  | LED    |
| :---: | :----: | ------ |
|   0   | 1 (LH) | 0=LED1 |
|   1   |   "    | 1=LED2 |
|   2   |   "    | 2=LED3 |
|   3   |   "    | 3=LED4 |
|   4   |   "    | 4=LED5 |
|   5   |   "    | 5=LED6 |
|   6   |   "    | 6=LED7 |
|   7   | 2 (LV) | 0=LED1 |
|   8   |   "    | 1=LED2 |
|   9   |   "    | ...    |
|  10   |   "    | ...    |
|  11   |   "    | ...    |
|  12   |   "    | ...    |
|  13   |   "    | 6=LED7 |
|  14   | 3 (RV) | 0=LED1 |
|  15   |   "    | 1=LED2 |
|  16   |   "    | ...    |
|  17   |   "    | ...    |
|  18   |   "    | ...    |
|  19   |   "    | ...    |
|  20   |   "    | 6=LED7 |
|  21   | 4 (RH) | 0=LED1 |
|  22   |   "    | 1=LED2 |
|  23   |   "    | ...    |
|  24   |   "    | ...    |
|  25   |   "    | ...    |
|  26   |   "    | ...    |
|  27   |   "    | 6=LED7 |



```
int32 pattern
string ledtype
int32 timeout
int32 duration
int32 duration_on
int32 duration_off
float32 brightness
int32 ledmask
```

# Neues Muster implementieren

## led_pattern.py
In diese ENUM muss die neue Pattern-ID eingetragen werden

## led_node.py
Unter self.pattern({}) muss analog zu den bestehenden Pattern ein neuer Eintrag hinzugefügt werden.
Die neue LEDPattern.xxxx nutzen die man im ENUM erstellt hat
ggf. muss eine Anpassung an der ledmask vorgenommen werden.
bei Bedarf in der Klasse LEDUtils eine neue Combination erstellen.

```
            LEDPattern.RED: LEDPatternConfig(LEDPattern.RED, 
                "FILL RED", 
                duration=0, 
                timeout=0, 
                callback='fill', 
                callback_param={
                    "color":(255, 0, 0),
                    "timeout":self.led_default_timeout, 
                    "duration_on": self.led_default_duration_on, 
                    "duration_off": self.led_default_duration_off, 
                    "brightness": self.led_default_brightness,
                    "ledmask" : LEDUtils.combination_mask('ALL')
                }
            ),
```
- callback = 'xxxxx' - hier muss der Funktionsname eingetragen werden der in ws2812 implementiert wird. Falls man mit fill nicht klar kommt
- callback_param - die sind entscheidend wie das Muster dargesetellt wird
- Sprechenden namen geben. Im Beispiel "FILL RED"


## LEDPatternConfig
keine Anpassung notwendig

## class WS2812 anpassen
wenn man tatsächlich ein komplett neues Muster implementieren möchte, muss dies hier in dieser Klasse implementiert werden, analog zu `fill` und `blink`.
Der Name der Funktion ist dann auch der name der als callback-funktion genutzt werden muss.


# Test
zwei Terminal-Fenster öffnen

**Terminal 1 - Rover-Projekt starten**
``

**Response aus Beispiel 1**
` `

**Response aus Beispiel 2**
` `

**Response aus Beispiel 3**
` `

**Response aus Beispiel 4**
` `

**Response aus Beispiel 5**
` `


**Terminal 2 - Testnachrichten in /led publishen**

**Beispiel 1**: Pattern 1 (GREEN), Timeout 5000, Duration 1000, LEDmask: default, --once 1 nachricht
`ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 1, ledtype: 'WS2812', timeout: 5000, duration_on: 1000}" --once`

**Beispiel 2**: Pattern  (0) (red), alles default, --once 1 nachricht
`ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 1, ledtype: 'WS2812'}" --once`

**Beispiel 2**: Pattern  (0) (red), alles default, --once 1 nachricht
`ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 1, ledtype: 'WS2812'}" --once`

**Beispiel 3**: Pattern  (0) (red), alles default, --once 1 nachricht
`ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 1, ledtype: 'WS2812'}" --once`

**Beispiel 4**: Pattern  (0) (red), alles default, --once 1 nachricht
`ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 1, ledtype: 'WS2812'}" --once`

**Beispiel 5**: Pattern  (0) (red), alles default, --once 1 nachricht
`ros2 topic pub /led rover_interfaces/msg/LEDMessage "{pattern: 1, ledtype: 'WS2812'}" --once`

