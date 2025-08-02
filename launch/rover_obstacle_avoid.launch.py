from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.event_handlers import OnShutdown
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.actions import LogInfo

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
    params_obstacle_avoid_node = os.path.join(share_dir, 'config', 'obstacle_avoid_node.yaml')
    params_common = os.path.join(share_dir, 'config', 'rover_common.yaml')


    # Liste der Lifecycle-Nodes (name, executable)
    lifecycle_nodes = [
        ('i2c_node', 'i2c_node', [], params_i2c_node),
        ('driver_controller_node', 'driver_controller_node', ['i2c_node'], params_driver_controller_node),
        ('sensor_node', 'sensor_node', [], params_sensor_node),
        ('obstacle_avoidance_node', 'obstacle_avoidance_node',[], params_obstacle_avoid_node),
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
        #return GroupAction(actions=nodes)
    

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
        #OpaqueFunction(function=create_driver_controller_node),  # <== ersetzt den alten driver_controller_node,
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
    base_delay = 3.0
    step = 1.5
    shutdown_step = 1.0
    lifecycle_nodes_group = []

    for idx, (node_name, executable, dependencies, param_file) in enumerate(lifecycle_nodes):
        lifecycle_nodes_group.append(
            LogInfo(msg=f"[Launch] ==> LifecycleNode {node_name} wird erstellt...")
        )

        lifecycle_nodes_group.append(
            LifecycleNode(
                package='rover',
                executable=executable,
                name=node_name,
                namespace='/',
                output='screen',
                parameters=[param_file]
            )
        )

        lifecycle_nodes_group.append(
            LogInfo(msg=f"[Launch] ==> LifecycleNode {node_name} on_configure...")
        )

        # on_configure()
        # Konfiguration aller Nodes zeitversetzt
        lifecycle_nodes_group.append(
            TimerAction(
                period=base_delay + idx * step,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'configure'],
                        output='screen'
                    )
                ]
            )
        )


        lifecycle_nodes_group.append(
            LogInfo(msg=f"[Launch] ==> LifecycleNode {node_name} on_activate...")
        )

        # on_activate()
        # Aktivierung, nachdem alle Abhängigkeiten geprüft wurden
        lifecycle_nodes_group.append(
            TimerAction(
                period=base_delay + idx * step + 2.0 + len(dependencies) * 1.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'activate'],
                        output='screen'
                    )
                ]
            )
        )


        lifecycle_nodes_group.append(
            LogInfo(msg=f"[Launch] ==> LifecycleNode {node_name} wait on dependencies...")
        )

        # Warten auf alle Abhängigkeiten
        for dep in dependencies:
            lifecycle_nodes_group.append(
                LogInfo(msg=f"[Launch] ==> Warte auf Dependency {dep} vor Aktivierung von {node_name}...")
            )

            lifecycle_nodes_group.append(
                TimerAction(
                    period=base_delay + idx * step + 1.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['python3', os.path.join(share_dir, 'scripts', 'wait_for_lifecycle.py'), dep],
                            output='screen'
                        )
                    ]
                )
            )

    # --- END FOR ----------------------------------------------------------------------


    # https://patorjk.com/software/taag/#p=display&f=Slant&t=ROVER%20PROJECT
    # Font: SLANT, Fitted, default height
    logo = r"""
   ____   ____  _____ ______ ___    ______ __     ______        
  / __ \ / __ )/ ___//_  __//   |  / ____// /    / ____/        
 / / / // __  |\__ \  / /  / /| | / /    / /    / __/           
/ /_/ // /_/ /___/ / / /  / ___ |/ /___ / /___ / /___           
\____//_____//____/ /_/  /_/  |_|\____//_____//_____/           
    ___  _    __ ____   ____ ____   ___     _   __ ______ ______
   /   || |  / // __ \ /  _// __ \ /   |   / | / // ____// ____/
  / /| || | / // / / / / / / / / // /| |  /  |/ // /    / __/   
 / ___ || |/ // /_/ /_/ / / /_/ // ___ | / /|  // /___ / /___   
/_/  |_||___/ \____//___//_____//_/  |_|/_/ |_/ \____//_____/   
  ______ ______ _____ ______                                    
 /_  __// ____// ___//_  __/                                    
  / /  / __/   \__ \  / /                                       
 / /  / /___  ___/ / / /                                        
/_/  /_____/ /____/ /_/                                         
                                                                                     
"""


    # 🔁 Rückgabe der LaunchDescription
    return LaunchDescription([
        LogInfo(msg=[logo, '\n\n']),
        lidar_model_arg,
        rviz_load_arg,
        core_nodes,
        nav_vision_nodes,
        *lifecycle_nodes_group,
        OpaqueFunction(function=create_nodes_from_arguments),
    ])