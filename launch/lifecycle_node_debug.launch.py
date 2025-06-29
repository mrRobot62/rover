
# launch/lifecycle_auto_start.launch.py
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Konfiguration
    rover_share_dir = get_package_share_directory('rover')
    param_file = os.path.join(rover_share_dir, 'config', 'rover.yaml')

    # Liste der Lifecycle-Nodes (name, executable)
    lifecycle_nodes = [
        ('lifecycle_node1', 'lifecycle_node1'),
        ('lifecycle_node2', 'lifecycle_node2'),
        ('odom_node', 'odom_node'),
        ('sensor_node', 'sensor_node'),
        ('vision_node', 'vision_node')
    ]

    # 🧱 LaunchDescription initialisieren
    ld = []

    # 🧼 Gestaffeltes Herunterfahren: deactivate → shutdown je Node
    shutdown_actions = []
    delay = 0.0  # Startverzögerung
    delay_step = 1.0  # Zeit zwischen jedem Befehl

    for node_name, _ in lifecycle_nodes:
        shutdown_actions.append(
            TimerAction(
                period=delay,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'deactivate'],
                        output='screen'
                    )
                ]
            )
        )
        delay += delay_step

        shutdown_actions.append(
            TimerAction(
                period=delay,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'shutdown'],
                        output='screen'
                    )
                ]
            )
        )
        delay += delay_step

    ld.append(
        RegisterEventHandler(
            OnShutdown(on_shutdown=shutdown_actions)
        )
    )

    # Node-Erzeugung + Timer für configure & activate
    base_delay = 3.0  # Startverzögerung
    step = 0.5        # Zeitabstand zwischen den Konfigurierungen

    for index, (node_name, executable) in enumerate(lifecycle_nodes):
        delay_config = base_delay + index * step
        delay_activate = delay_config + 2.0  # 2 Sekunden nach configure

        # LifecycleNode hinzufügen
        ld.append(
            LifecycleNode(
                package='rover',
                executable=executable,
                name=node_name,
                namespace='/',
                output='screen',
                parameters=[param_file]
            )
        )

        # Konfigurierungs-Timer
        ld.append(
            TimerAction(
                period=delay_config,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'configure'],
                        output='screen'
                    )
                ]
            )
        )

        # Aktivierungs-Timer
        ld.append(
            TimerAction(
                period=delay_activate,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'activate'],
                        output='screen'
                    )
                ]
            )
        )

        # 💥 Shutdown-Hook zum Ausführen des Skripts



    return LaunchDescription(ld)

