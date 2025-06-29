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
    urdf_config_file = os.path.join(share_dir, 'config/urdf', 'rover.urdf')
    params_file = os.path.join(share_dir, 'config', 'rover.yaml')

    # Liste der Lifecycle-Nodes (name, executable)
    lifecycle_nodes = [
        ('odom_node', 'odom_node'),
        ('sensor_node', 'sensor_node'),
        ('vision_node', 'vision_node')
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

    lidar_model = LaunchConfiguration('lidar_model')
    lidar_topic = LaunchConfiguration('lidar_topic', default='/scan')

    sensor_node = LifecycleNode(
        package='rover',
        executable='sensor_node',
        name='sensor_node',
        output='screen',
        namespace='/',
        parameters=[
            LaunchConfiguration('params_file'),
            {'lidar_topic': lidar_topic}
        ]
    )

    # wird nur dann benötigt, wenn ich direkt ein Node erstellen möchte
    #lidar_model = LaunchConfiguration('lidar_model')

    #
    # den Lidar basierend auf den Aufrufparameter kann nicht direkt genutzt und ausgewertet werden.
    # daher hier ein Wrapper und später die OpaqueFunction()
    #
    def create_lidar_node(context):
        model = context.launch_configurations['lidar_model']
        params = load_rover_params(params_file, model)

        nodes = []

        if model == 'ydlidar':
            logger.info("Create YDLidar Node -> [{params}]")
            ydlidar_launch_file = params.get('ydlidar_launch', 'full_lidar.launch.py')
            ydlidar_param_file = params.get('ydlidar_param', 'TminiPlus.yaml')

            ydlidar_launch_path = os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                ydlidar_launch_file
            )

            ydlidar_param_path = os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'params', ydlidar_param_file
            )

            logger.info(f"[rover_launch] Model: {model}")
            logger.info(f"[rover_launch] Launch: {ydlidar_launch_path}")
            logger.info(f"[rover_launch] Param: {ydlidar_param_path}")

            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ydlidar_launch_path),
                launch_arguments={
                    'params_file' : ydlidar_param_path
                }.items()

            ))

        elif model == 'xv11':
            logger.info("Create XV11Lidar Node -> [{params}]")
            nodes.append(
                Node(
                    package='xv11_lidar_python',
                    executable='xv11_lidar',
                    name='xv11_lidar_node',
                    output='screen',
                    parameters=[params]
                )
            )

        
        return nodes

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

    # 🧾 Launch-Argumente
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Pfad zur Parameterdatei des Rovers'
    )




    """
    Transform-Node. Dieser Node Transformiert die Welt-Koordinaten auf base_link
    Das wird benötigt um später z.b SLAM nutzen zu können

    """
    tf2_world_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='world_to_base_link',
                    arguments=[
                        '--x', '0.0',
                        '--y', '0.0',
                        '--z', '0.0',
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'world',
                        '--child-frame-id', 'base_link',
                    ],                    
                    )

    """
    Transform-Node. Dieser Node Transformiert die Sensordaten aus dem Frame 'laser_frame' (siehe TminiPro.yaml) in base_link
    base_link symbolisiert die physische Verortung des Sensors auf dem Roboter.

    """
    tf2_base_link_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link_to_laser_frame',
                    arguments=[
                        '--x', '0.10',
                        '--y', '0.0',
                        '--z', '0.15',
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'laser_frame',
                    ],                    
                    )

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
                parameters=[params_file]
            )
        ]

    # 🧭 Navigation Node
    navigation_node = Node(
        package='rover',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        #parameters=[LaunchConfiguration('params_file')]
        parameters=[params_file]
    )

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
        parameters=[params_file]
    )

    #--------------------------------------------------------------------------------------
    # LifeCycle Nodes und Management
    #--------------------------------------------------------------------------------------
    lifecycle_node_definitions = GroupAction([
        LogInfo(msg='[Launch] Starte LifecycleNodes...'),
        LifecycleNode(
            package='rover',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            namespace='/',
            parameters=[params_file]
        ),
        LifecycleNode(
            package='rover',
            executable='odom_node',
            name='odom_node',
            output='screen',
            namespace='/',
            parameters=[params_file]
        ),
        LifecycleNode(
            package='rover',
            executable='vision_node',
            name='vision_node',
            output='screen',
            namespace='/',
            parameters=[params_file]
        ),
    ])

    lifecycle_status_marker_node = Node(
        package='rover',
        executable='lifecycle_status_marker',
        name='lifecycle_status_marker',
        output='screen'
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
        led_node,
        tf2_world_node,
        tf2_base_link_node,
        OpaqueFunction(function=create_driver_controller_node),  # <== ersetzt den alten driver_controller_node,
        gamepad_nodes
    ])

    nav_vision_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Navigation und Vision...'),
        navigation_node,
    ])


    # Lifecycle-Steuerung (Configure + Activate + Shutdown)
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
        lifecycle_status_marker_node,
        *lifecycle_startup
    ])

    logo = """
    ____   ____  _____ ___      ____   ____  _    __ ______ ____ 
   / __ \ / __ \/ ___/|__ \    / __ \ / __ \| |  / // ____// __ \ 
  / /_/ // / / /\__ \ __/ /   / /_/ // / / /| | / // __/  / /_/ /
 / _, _// /_/ /___/ // __/   / _, _// /_/ / | |/ // /___ / _, _/ 
/_/ |_| \____//____//____/  /_/ |_| \____/  |___//_____//_/ |_|  
    ____                  _              __                      
   / __ \ _____ ____     (_)___   _____ / /_                     
  / /_/ // ___// __ \   / // _ \ / ___// __/                     
 / ____// /   / /_/ /  / //  __// /__ / /_                       
/_/    /_/    \____/__/ / \___/ \___/ \__/                       
                   /___/                                         
"""

    # 🔁 Rückgabe der LaunchDescription
    return LaunchDescription([
        LogInfo(msg=[logo, '\n\n']),
        lidar_model_arg,
        rviz_load_arg,
        params_file_arg,
        OpaqueFunction(function=create_lidar_node),
        core_nodes,
        lifecycle_node_definitions,
        nav_vision_nodes,
        lifecycle_nodes_group,
        lifecycle_shutdown_handler,    
        OpaqueFunction(function=create_nodes_from_arguments)
    ])