import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Startet die Pi-Kamera (camera_ros)'
    )

    config_file = LaunchConfiguration('config_file')

    # System-libcamera vor /opt/ros/lyrical/lib laden, damit Pi-5 rp1_cfe_downstream erkannt wird
    sys_lib_dir = '/usr/lib/aarch64-linux-gnu'
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    camera_ld_path = f'{sys_lib_dir}:{existing_ld}' if existing_ld else sys_lib_dir

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
            executable='servo_node',
            name='servo_node',
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
        Node(
            package='picar_drivers',
            executable='battery_node',
            name='battery_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[config_file],
            additional_env={'LD_LIBRARY_PATH': camera_ld_path},
            condition=IfCondition(LaunchConfiguration('launch_camera'))
        ),

        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[config_file]
        ),

        # Teleop Node
        Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[config_file]
        ),
    ]

    return LaunchDescription([config_arg, launch_camera_arg] + nodes)

