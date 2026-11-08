import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('picar_drivers')
    default_config = os.path.join(pkg_share, 'config', 'picar_params.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Pfad zur zentralen YAML-Konfiguration fuer die PiCar Treiber'
    )

    config_file = LaunchConfiguration('config_file')

    nodes = [
        Node(
            package='picar_drivers',
            executable='led_node',
            name='led_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='picar_drivers',
            executable='buzzer_node',
            name='buzzer_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='picar_drivers',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='picar_drivers',
            executable='oled_node',
            name='oled_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='picar_drivers',
            executable='motor_node',
            name='motor_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='picar_drivers',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
            parameters=[config_file]
        ),
    ]

    return LaunchDescription([config_arg] + nodes)

