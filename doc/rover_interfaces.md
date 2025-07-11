# Rover_Interfaces
ist ein Projekt des rover-projektes. Hier werden rover spezifische Message-Strukturn
beschrieben

# Installation
Projekt einfach downloaden und in den `ros2_ws/src` Folder kopieren und dann 
von dort kompilieren.

# Kompilieren
Das erste mal wird das kompilieren vermutlich absolut fehlerfrei verlaufen, ändert man
aber im Laufe der Zeit etwas an den Messages und man muss neu kompilieren können kleinere Probleme
auftreten.
Nachfolgend einfache Lösungsmöglichkeiten

- `--symlink-install` grundsätzlich nicht verwenden

**Build**
`colcol build --packages-select rover_interfaces`


## Probleme:

**Fehler**
```
Starting >>> rover_interfaces
stderr: rover_interfaces                         
failed to create symbolic link '/home/bernd/ros2_ws/build/rover_interfaces/ament_cmake_python/rover_interfaces/rover_interfaces' because existing path cannot be removed: Is a directory
gmake[2]: *** [CMakeFiles/ament_cmake_python_symlink_rover_interfaces.dir/build.make:70: CMakeFiles/ament_cmake_python_symlink_rover_interfaces] Error 1
```

**Lösung**
im `build` und `install` folder einfach alles löschen was von rover_interfaces kommt

`rm -rf build/rover_interfaces`
`rm -rf install/rover_interfaces`

anschließend neu kompilieren

`colcol build --packages-select rover_interfaces`


Typische Logausgabe:
```
 2 bernd@ros2pi5:~/ros2_ws$ rm -rf build/rover_interfaces/
bernd@ros2pi5:~/ros2_ws$ rm -rf install/rover_interfaces/
bernd@ros2pi5:~/ros2_ws$ colcon build  --packages-select rover_interfaces
[0.335s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/bernd/ros2_ws/install/rover_interfaces' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.336s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/home/bernd/ros2_ws/install/rover_interfaces' in the environment variable CMAKE_PREFIX_PATH doesn't exist
Starting >>> rover_interfaces
Finished <<< rover_interfaces [16.6s]                       

Summary: 1 package finished [16.9s]
bernd@ros2pi5:~/ros2_ws$ 
```

# Prüfen ob die Messages verfügbar sind

Prüfen ob die erstelle Message auch verfügbar ist
` ros2 interface show rover_interfaces/msg/LEDMessage`
