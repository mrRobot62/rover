from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config-Dateien (z. B. YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Konfigurationen für rviz & urdf
        (os.path.join('share', package_name, 'config/rviz'), glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'config/urdf'), glob('config/urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LunaX',
    maintainer_email='berndklein@gmx.de',
    description='Allrad-Rover Projekt mit ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_controller_node = rover.driver_controller_node:main',
            'sensor_node = rover.sensor_node:main',
            'odom_node = rover.odom_node:main',
            'navigation_node = rover.navigation_node:main',
            'vision_node = rover.vision_node:main',
            'lifecycle_node1=rover.lifecycle_node1:main',
            'lifecycle_node2=rover.lifecycle_node2:main',
            'led_node = rover.led_node:main',
            'lifecycle_dashboard = rover.lifecycle_dashboard:main',
            #'lifecycle_manager = rover.lifecycle_manager:main',
            'lifecycle_status_marker = rover.lifecycle_status_marker:main', 
        ],
    },
)
