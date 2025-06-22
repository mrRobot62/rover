from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction

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

    """
    Lidar-Parameter:
        lidar_model:= <LidarModel> in rover.yaml

    """
    lidar_launch_args = DeclareLaunchArgument(
        'lidar_model',
        default_value='ydlidar',
        description="Auswahl des Lidar-Models: ydlidar oder xv11"
    )

    # 🧾 Launch-Argumente
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Pfad zur Parameterdatei des Rovers'
    )
    
    # 👀 Soll rviz2 genutzt werden? default=True
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz starten oder nicht'
    )

    # 🧾 Lifecycle Marker Node (zeigt den Zustand von odom_node in RViz an)
    lifecycle_status_marker_node = Node(
        package='rover',
        executable='lifecycle_status_marker',
        name='lifecycle_status_marker',
        output='screen'
    )

    # 📡 Sensor Node
    sensor_node = Node(
        package='rover',
        executable='sensor_node',
        name='sensor_node',
        output='screen',
        #parameters=[LaunchConfiguration('params_file')]
    )

    # 🚗 Drive Controller Node
    drive_controller_node = Node(
        package='rover',
        executable='drive_controller_node',
        name='drive_controller_node',
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

    # 👀 RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 📦 Gruppenbildung
    core_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Sensorik und Steuerung...'),
        sensor_node,
        drive_controller_node,
        odom_node,
#        lifecycle_manager,
#        lifecycle_status_marker_node 

    ])

    nav_vision_nodes = GroupAction([
        LogInfo(msg='[Launch] Starte Navigation und Vision...'),
        navigation_node,
        vision_node
    ])



    # 🔁 Rückgabe der LaunchDescription
    return LaunchDescription([
        params_file_arg,
        lidar_model_arg,
        use_rviz_arg,
        core_nodes,
        nav_vision_nodes,
#        rviz_node
    ])