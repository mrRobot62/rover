
# GIT Terminal-Befehle
| Git-Befehl                    | Beispiel                                     | Informationen                                                              | Typischer Anwendungsfall                           |
| ----------------------------- | -------------------------------------------- | -------------------------------------------------------------------------- | -------------------------------------------------- |
| `git init`                    | `git init`                                   | Initialisiert ein neues Git-Repository im aktuellen Ordner                 | Start eines neuen Projekts unter Versionskontrolle |
| `git clone <url>`             | `git clone https://github.com/user/repo.git` | Klont ein entferntes Repository auf den lokalen Rechner                    | Bestehendes Projekt herunterladen                  |
| `git status`                  | `git status`                                 | Zeigt den aktuellen Status des Repositories an (Änderungen, Commits, etc.) | Überblick über Arbeitsverzeichnis                  |
| `git add <datei>`             | `git add index.html`                         | Stellt eine Datei für den nächsten Commit bereit                           | Änderungen vorbereiten                             |
| `git add .`                   | `git add .`                                  | Fügt alle Änderungen im aktuellen Verzeichnis hinzu                        | Alle neuen/geänderten Dateien aufnehmen            |
| `git commit -m "<nachricht>"` | `git commit -m "Initial commit"`             | Speichert die Änderungen im lokalen Repository mit Nachricht               | Versionspunkt setzen                               |
| `git log`                     | `git log`                                    | Zeigt die Commit-Historie an                                               | Verlauf ansehen                                    |
| `git diff`                    | `git diff`                                   | Zeigt Unterschiede zwischen Arbeitsverzeichnis und letztem Commit          | Änderungen im Detail überprüfen                    |
| `git branch`                  | `git branch`                                 | Zeigt vorhandene Branches an                                               | Überblick über Zweige bekommen                     |
| `git branch <name>`           | `git branch feature-x`                       | Erstellt einen neuen Branch                                                | Entwicklung eines neuen Features                   |
| `git checkout <name>`         | `git checkout feature-x`                     | Wechselt zu einem anderen Branch                                           | Zu einer Funktion oder Version wechseln            |
| `git checkout -b <name>`      | `git checkout -b feature-x`                  | Erstellt und wechselt zu einem neuen Branch gleichzeitig                   | Schnell neue Entwicklung starten                   |
| `git merge <branch>`          | `git merge feature-x`                        | Integriert einen Branch in den aktuellen Branch                            | Feature in Hauptentwicklung übernehmen             |
| `git pull`                    | `git pull`                                   | Holt und integriert Änderungen vom Remote-Repository                       | Projekt auf neuesten Stand bringen                 |
| `git fetch`                   | `git fetch`                                  | Holt Änderungen vom Remote, integriert sie aber nicht automatisch          | Änderungen prüfen, ohne sie sofort zu übernehmen   |
| `git push`                    | `git push`                                   | Überträgt lokale Commits ins Remote-Repository                             | Änderungen veröffentlichen                         |
| `git remote -v`               | `git remote -v`                              | Zeigt konfigurierten Remote-Server an                                      | Kontrolle über verbundene Remotes                  |
| `git remote add <name> <url>` | `git remote add origin https://...`          | Fügt ein neues Remote-Repository hinzu                                     | Verbindung zu GitHub/GitLab herstellen             |
| `git reset --hard`            | `git reset --hard HEAD`                      | Setzt alle Änderungen auf letzten Commit zurück                            | Arbeitsverzeichnis bereinigen                      |
| `git stash`                   | `git stash`                                  | Speichert temporär Änderungen, ohne sie zu committen                       | Zwischenstand sichern                              |
| `git stash pop`               | `git stash pop`                              | Holt zuletzt gespeicherte Änderungen aus dem Stash                         | Temporär zurückgelegte Änderungen wiederherstellen |
| `git tag <version>`           | `git tag v1.0.0`                             | Erstellt einen Versions-Tag                                                | Release-Version markieren                          |
| `git revert <commit>`         | `git revert a1b2c3d`                         | Macht einen Commit rückgängig durch einen neuen Commit                     | Fehlerhaften Commit rückgängig machen              |
| `git rm <datei>`              | `git rm old_file.txt`                        | Entfernt eine Datei aus dem Git-Repository                                 | Datei aus Versionskontrolle entfernen              |
| `git mv <alt> <neu>`          | `git mv alt.txt neu.txt`                     | Benennt eine Datei um und verfolgt Änderung                                | Dateien umstrukturieren                            |



# Git-Ignore
| Muster/Eintrag   | Beispiel         | Informationen                                             | Typischer Anwendungsfall                                |
| ---------------- | ---------------- | --------------------------------------------------------- | ------------------------------------------------------- |
| `*.log`          | `*.log`          | Ignoriert alle `.log`-Dateien im gesamten Projekt         | Logdateien aus Entwicklung ausschließen                 |
| `build/`         | `build/`         | Ignoriert den gesamten Ordner `build/`                    | Temporäre Build-Artefakte nicht versionieren            |
| `*.tmp`          | `*.tmp`          | Ignoriert alle temporären Dateien mit `.tmp`-Endung       | Temporäre Dateien vom Editor oder Compiler ausschließen |
| `!important.txt` | `!important.txt` | Hebt einen vorher ignorierten Eintrag explizit wieder auf | Eine bestimmte Datei dennoch versionieren               |
| `/config.yaml`   | `/config.yaml`   | Ignoriert nur die Datei im Projekt-Wurzelverzeichnis      | Lokale Konfigurationen schützen                         |
| `**/debug.log`   | `**/debug.log`   | Ignoriert `debug.log` in allen Unterverzeichnissen        | Debug-Dateien projektweit ausschließen                  |
| `*.pyc`          | `*.pyc`          | Ignoriert alle Python-Bytecode-Dateien                    | Kompilierte Dateien vermeiden                           |
| `.env`           | `.env`           | Ignoriert Umgebungsvariablen-Dateien                      | Vermeidung sensibler Informationen                      |
| `node_modules/`  | `node_modules/`  | Ignoriert den Node.js-Dependencies-Ordner                 | Paketabhängigkeiten nicht committen                     |
| `dist/`          | `dist/`          | Ignoriert generierte Distributionsdateien                 | Ausgaben von Build-Tools nicht versionieren             |
| `*.swp`          | `*.swp`          | Ignoriert Swap-Dateien von Vim                            | Editor-spezifische Dateien ausschließen                 |
| `.idea/`         | `.idea/`         | Ignoriert JetBrains IDE-Projektdateien                    | Projekt-Einstellungen nicht teilen                      |
| `__pycache__/`   | `__pycache__/`   | Ignoriert Python-Cache-Ordner                             | Automatisch generierte Ordner vermeiden                 |
| `*.bak`          | `*.bak`          | Ignoriert Backup-Dateien                                  | Alte Sicherungen vom Editor nicht committen             |
| `*.DS_Store`     | `*.DS_Store`     | Ignoriert macOS-spezifische Metadaten-Dateien             | Cross-Plattform-Kompatibilität                          |