from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution

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
    default_param_file = os.path.join(share_dir, 'config', 'rover.yaml')


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

    # wird nur dann benötigt, wenn ich direkt ein Node erstellen möchte
    #lidar_model = LaunchConfiguration('lidar_model')

    #
    # den Lidar basierend auf den Aufrufparameter kann nicht direkt genutzt und ausgewertet werden.
    # daher hier ein Wrapper und später die OpaqueFunction()
    #
    def create_lidar_node(context):
        model = context.launch_configurations['lidar_model']
        params = load_rover_params(default_param_file, model)

        nodes = []

        # 📡 Sensor Node
        lidar_topic = params.get('lidar_topic', '/scan')
        sensor_node = Node(
            package='rover',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[{
                'lidar_topic': TextSubstitution(text=lidar_topic)
            }]
        )
        nodes.append(sensor_node)

        if model == 'ydlidar':
            logger.info("Create YDLidar -> [{params}]")
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
            logger.info("Create XV11Lidar -> [{params}]")
            return [Node(
                package='xv_11_laser_driver',
                executable='xv_11_laser_node',
                name='xv11_node',
                parameters=[params]
            )]
        
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
        default_value=default_param_file,
        description='Pfad zur Parameterdatei des Rovers'
    )


    # 🧾 Lifecycle Marker Node (zeigt den Zustand von odom_node in RViz an)
    lifecycle_status_marker_node = Node(
        package='rover',
        executable='lifecycle_status_marker',
        name='lifecycle_status_marker',
        output='screen'
    )

    # # 📡 Sensor Node
    # sensor_node = Node(
    #     package='rover',
    #     executable='sensor_node',
    #     name='sensor_node',
    #     output='screen',
    #     parameters=[{
    #         'lidar_topic': TextSubstitution(text=lidar_topic)
    #     }]
    # )

    # 🚗 Drive Controller Node
    driver_controller_node = Node(
        package='rover',
        executable='driver_controller_node',
        name='driver_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # 🧭 Navigation Node
    navigation_node = Node(
        package='rover',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # 🔄 Odometrie Node (als LifecycleNode)
    odom_node = LifecycleNode(
        package='rover',
        executable='odom_node',
        name='odom_node',
        output='screen',
        namespace='/',
        parameters=[LaunchConfiguration('params_file')]
    )

    # ⚙️ Lifecycle Manager für den odom_node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_odom',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['odom_node'],
            'bond_timeout': 0.0
        }]
    )

    # 🎥 Vision Node
    vision_node = Node(
        package='rover',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

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

    # 📦 Gruppenbildung
    core_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Sensorik und Steuerung...'),
        #sensor_node,
        #manual_control_node,
        driver_controller_node,
        odom_node,
#        lifecycle_manager,
#        lifecycle_status_marker_node 
        gamepad_nodes
    ])

    nav_vision_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Navigation und Vision...'),
        navigation_node,
        vision_node
    ])
 

    # 🔁 Rückgabe der LaunchDescription
    return LaunchDescription([
        lidar_model_arg,
        rviz_load_arg,
        params_file_arg,
        OpaqueFunction(function=create_lidar_node),
        core_nodes,
        nav_vision_nodes,
        OpaqueFunction(function=create_nodes_from_arguments)
    ])