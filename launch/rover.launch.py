from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os



def generate_launch_description():
    share_dir = get_package_share_directory('rover')
    rviz_config_file = os.path.join(share_dir, 'config', 'rover.rviz')
    param_file = LaunchConfiguration('params_file')
    node_name = 'rover_node'


    """
    Definition und Einbindung der Rover-parameter-Datei. Diese Datei enthält Basiskonfiguration
    zum Start des Rovers
    """
    rover_params_declare = LaunchConfiguration(
        'params_file', 
        default=os.path.join(share_dir,'params', 'rover.yaml'),
        description='Pfad zum rover.yaml Parameterdatei'
    )

    """ 
    Sensor_Node, Sensor-Fusion-Node, verarbeitet alle Sensordaten
    """
    sensor_node =  Node(
        package='rover', 
        executable='sensor_node', 
        name='sensor_node', 
        output='screen'
    )

    """ 
    drive_controller_node : Ist für die Radsteuerung (Velocity & Steering) verantwortlich
    """
    driver_controller_node = Node(
        package='rover', 
        executable='drive_controller_node', 
        name='drive_controller_node', 
        output='screen'
    )

    """ 
    odom_node: Ist für die odometrie verantwortlich
    """
    odom_node = Node(
        package='rover', 
        executable='odom_node', 
        name='odom_node', 
        output='screen'
    )

    """ 
    navigation_node: ist für die navigation des Rovers verantwortlich, nutzt SLAM
    """
    navigtion_node = Node(
        package='rover', 
        executable='navigation_node', 
        name='navigation_node', output='screen'
    )

    """ 
    vision_node: ist für die Verarbeitung des kamerabildes/-streams verantwortlich
    """
    vision_node = Node(
        package='rover', 
        executable='vision_node', 
        name='vision_node', output='screen'
    )

    """
    Gibt die kompletten erforderlichen Komponenten zurück um zu Starten
    """
    return LaunchDescription([
        rover_params_declare,
        sensor_node,
        driver_controller_node,
        odom_node,
        navigtion_node,
        vision_node
    ])
