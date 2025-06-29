# Tips & Tricks für die Entwicklung

## Logausgaben farbig ausgeben
| Farbe       | ANSI-Code  | Beispiel                             |
| ----------- | ---------- | ------------------------------------ |
| Schwarz     | `\033[30m` | `\033[30mText\033[0m`                |
| **Rot**     | `\033[31m` | `\033[31mText\033[0m`                |
| **Grün**    | `\033[32m` | `\033[32mText\033[0m`                |
| **Gelb**    | `\033[33m` | `\033[33mText\033[0m`                |
| **Blau**    | `\033[34m` | `\033[34mText\033[0m`                |
| **Magenta** | `\033[35m` | `\033[35mText\033[0m`                |
| **Cyan**    | `\033[36m` | `\033[36mText\033[0m`                |
| Weiß        | `\033[37m` | `\033[37mText\033[0m`                |
| **Reset**   | `\033[0m`  | Setzt Farbe zurück auf Standardfarbe |

`self.get_logger().info("\033[34mDies ist blau\033[0m")`

**WICHTIG** `\033[0m` muss immer am Ende des String hinzugefügt werden, sonst bleibt die Terminalausgabe immer farbig


