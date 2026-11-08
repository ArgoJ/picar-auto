import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Bevorzuge die editierbare Konfiguration im src-Verzeichnis, falls vorhanden
    src_config = '/home/pirob/ros2_ws/src/picar_drivers/config/picar_params.yaml'
    if os.path.exists(src_config):
        default_config = src_config
    else:
        pkg_share = get_package_share_directory('picar_drivers')
        default_config = os.path.join(pkg_share, 'config', 'picar_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Pfad zur zentralen YAML-Konfiguration fuer PiCar Treiber und Kalibrierung'
    )

    config_file = LaunchConfiguration('config_file')

    # 1. Hardware-Treiber fuer Motoren und Servos (I2C PCA9685)
    motor_node = Node(
        package='picar_drivers',
        executable='motor_node',
        name='motor_node',
        output='screen',
        parameters=[config_file]
    )

    # 2. Kalibrierungs-Service (verwaltet Offsets & Min/Max, speichert in YAML)
    kalibration_service = Node(
        package='picar_kalibration',
        executable='kalibration_service_node',
        name='kalibration_service_node',
        output='screen',
        parameters=[{'config_file': default_config}]
    )

    return LaunchDescription([
        config_arg,
        motor_node,
        kalibration_service
    ])
