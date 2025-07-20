from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown
from launch.actions import RegisterEventHandler, ExecuteProcess

import launch.logging

import os
import yaml

def load_rover_params(path: str, lidar_model:str):
    with open(path,'r') as f:
        data = yaml.safe_load(f)
        return data.get(lidar_model, {}).get("ros__parameters", {})

def generate_launch_description():
    logger = launch.logging.get_logger('rover_launch')
    logger.info("Starte rover_full.launch.py...")
    # 📁 Paketverzeichnis
    share_dir = get_package_share_directory('rover')
    rviz_config_file = os.path.join(share_dir, 'config/rviz', 'rover.rviz')
    params_i2c_node = os.path.join(share_dir, 'config', 'i2c_node.yaml')
    params_sensor_node = os.path.join(share_dir, 'config', 'sensor_node.yaml')
    params_driver_controller_node = os.path.join(share_dir, 'config', 'driver_controller_node.yaml')
    params_led_node = os.path.join(share_dir, 'config', 'led_node.yaml')

    params_common = os.path.join(share_dir, 'config', 'rover_common.yaml')


    # Liste der Lifecycle-Nodes (name, executable)
    lifecycle_nodes = [
        ('i2c_node', 'i2c_node'),
        ('sensor_node', 'sensor_node'),
    ]

    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='ydlidar',
        description="Auswahl des Lidar-Models: ydlidar oder xv11"
    )

    rviz_load_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz starten oder nicht'
    )

    def create_nodes_from_arguments(context):
        nodes = []
        if LaunchConfiguration('use_rviz').perform(context) == 'true':
            # 👀 RViz (optional)
            nodes.append(
                Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                arguments=['-d', rviz_config_file],
                output='screen'
            ))

        return nodes

    # 🚗 Drive Controller Node
    # wenn man so in der Launch den Node deklariert.
    # muss man innerhalb des Nodes die Parameter nicht vorher deklarieren
    # sondern kann sie direkt auslesen.
    # Nachteil ist ein Parameter nicht vorhanden, wird ein Fehler geloggt und der
    # Node startet nicht
    # Vorteil: deutlich einfacher innerhalb des Nodes

    def create_driver_controller_node(context):
        return [
            Node(
                package='rover',
                executable='driver_controller_node',
                name='driver_controller_node',
                output='screen',
                parameters=[params_driver_controller_node]
            )
        ]

    # LED Node
    #
    # wenn man so in der Launch den Node deklariert.
    # muss man innerhalb des Nodes die Parameter vorher deklarieren
    # erst dann kann man sie auslesen
    # 
    # Vorteil: sollte ein Parameter nicht vorhanden sein, kann man default werten arbeiten (sicherer)
    # Nachteil: mehr Code im Node
    led_node = Node(

        package='rover',
        executable='led_node',
        name='led_node',
        output='screen',
        parameters=[params_led_node]
    )

    #--------------------------------------------------------------------------------------
    # LifeCycle Nodes und Management
    #--------------------------------------------------------------------------------------
    lifecycle_node_definitions = GroupAction([
        LogInfo(msg='[Launch] Starte LifecycleNodes...'),
        LifecycleNode(
            package='rover',
            executable='i2c_node',
            name='i2c_node',
            output='screen',
            namespace='/',
            parameters=[params_i2c_node]
        ),
    ])

    #--------------------------------------------------------------------------------------


    # 🕹️ Gamepad Steuerung über teleop_twist_joy
    teleop_twist_joy_launch_path = os.path.join(
        get_package_share_directory('teleop_twist_joy'),
        'launch',
        'teleop-launch.py'
    )

    gamepad_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Gamepad Steuerung über teleop_twist_joy...'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_twist_joy_launch_path),
            launch_arguments={
                'config': 'xbox'
            }.items()
        )
    ])

    # 📦 Gruppenbildung - 
    core_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Sensorik und Steuerung...'),
        OpaqueFunction(function=create_driver_controller_node),  # <== ersetzt den alten driver_controller_node,
        led_node,
        gamepad_nodes
    ])

    nav_vision_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Navigation und Vision...'),
    ])


    # Lifecycle-Steuerung (Configure + Activate + Shutdown)
    #
    # Dieses verfahren habe ich gewählt, weil ich nav2_lifecycle_manager nicht zum Laufen gebracht habe
    # die Transitionsübergänge haben nie automatisch funktioniert sondern immer nur manuell
    # Das führte dazu, das beim Start des roverprojektes die LifeCycleNotes nie im active-State waren
    #
    # Nachfolgend eine einfacher manueller Ansatz der die Statusübergänge automatisiert.
    lifecycle_startup = []
    lifecycle_shutdown = []
    base_delay = 3.0
    step = 0.5
    shutdown_step = 1.0

    for idx, (name, _) in enumerate(lifecycle_nodes):
        delay_config = base_delay + idx * step
        delay_activate = delay_config + 2.0
        shutdown_delay1 = idx * shutdown_step
        shutdown_delay2 = shutdown_delay1 + shutdown_step

        # Configure
        lifecycle_startup.append(
            TimerAction(
                period=delay_config,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{name}', 'configure'],
                        output='screen'
                    )
                ]
            )
        )

        # Activate
        lifecycle_startup.append(
            TimerAction(
                period=delay_activate,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{name}', 'activate'],
                        output='screen'
                    )
                ]
            )
        )

        # Shutdown: Deactivate → Shutdown
        lifecycle_shutdown.append(
            TimerAction(
                period=shutdown_delay1,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{name}', 'deactivate'],
                        output='screen'
                    )
                ]
            )
        )

        lifecycle_shutdown.append(
            TimerAction(
                period=shutdown_delay2,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{name}', 'shutdown'],
                        output='screen'
                    )
                ]
            )
        )
    # --- END FOR ----------------------------------------------------------------------


    # EventHandler für OnShutdown
    lifecycle_shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=lifecycle_shutdown)
    )

 
    lifecycle_nodes_group = GroupAction([
        LogInfo(msg='[Launch] Initialisiere Lifecycle-Aktionen...'),
        *lifecycle_startup
    ])

    # https://patorjk.com/software/taag/#p=display&f=Slant&t=ROVER%20PROJECT
    # Font: SLANT, Fitted, default height
    logo = """
    ____  ____  _____    ____________     __________  _   ____________  ____  __    __    __________ 
   / __ \/ __ \/  _/ |  / / ____/ __ \   / ____/ __ \/ | / /_  __/ __ \/ __ \/ /   / /   / ____/ __ \
  / / / / /_/ // / | | / / __/ / /_/ /  / /   / / / /  |/ / / / / /_/ / / / / /   / /   / __/ / /_/ /
 / /_/ / _, _// /  | |/ / /___/ _, _/  / /___/ /_/ / /|  / / / / _, _/ /_/ / /___/ /___/ /___/ _, _/ 
/_____/_/ |_/___/  |___/_____/_/ |_|   \____/\____/_/ |_/ /_/ /_/ |_|\____/_____/_____/_____/_/ |_|  
  _______________________                                                                            
 /_  __/ ____/ ___/_  __/                                                                            
  / / / __/  \__ \ / /                                                                               
 / / / /___ ___/ // /                                                                                
/_/ /_____//____//_/                       
"""


    # 🔁 Rückgabe der LaunchDescription
    return LaunchDescription([
        LogInfo(msg=[logo, '\n\n']),
        lidar_model_arg,
        rviz_load_arg,
        core_nodes,
        lifecycle_node_definitions,
        nav_vision_nodes,
        lifecycle_nodes_group,
        lifecycle_shutdown_handler,    
        OpaqueFunction(function=create_nodes_from_arguments),
    ])