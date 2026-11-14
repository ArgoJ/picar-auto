#!/usr/bin/env python3
"""
picar.launch.py - Zentraler Systemstart fuer PiCar-Treiber, Teleop und Roboterarm
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    drivers_share = get_package_share_directory('picar_drivers')
    arm_launch_share = get_package_share_directory('picar_arm_launch')

    default_config = os.path.join(drivers_share, 'config', 'picar_params.yaml')
    try:
        default_safety_config = os.path.join(
            get_package_share_directory('picar_arm_safety'),
            'config',
            'safety_params.yaml',
        )
        if not os.path.isfile(default_safety_config):
            default_safety_config = ''
    except PackageNotFoundError:
        default_safety_config = ''

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Pfad zur zentralen YAML-Konfiguration fuer picar_drivers und Teleop'
    )

    safety_config_arg = DeclareLaunchArgument(
        'safety_config_file',
        default_value=default_safety_config,
        description='Pfad zur YAML-Konfiguration fuer picar_arm_safety_node'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log-Level fuer den Roboterarm (debug, info, warn, error)'
    )

    launch_drivers_arg = DeclareLaunchArgument(
        'launch_drivers',
        default_value='true',
        description='Startet picar_drivers (inkl. motor_node, servo_node, Sensoren & joy/teleop)'
    )

    launch_arm_arg = DeclareLaunchArgument(
        'launch_arm',
        default_value='true',
        description='Startet das PiCar-Arm System (picar_arm_safety + arm_handler_node)'
    )

    launch_kalibration_arg = DeclareLaunchArgument(
        'launch_kalibration',
        default_value='false',
        description='Optional: Startet zusaetzlich kalibration_service_node'
    )

    # 1. Alle PiCar Hardware-Treiber & Controller (inkl. motor_node & servo_node)
    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drivers_share, 'launch', 'drivers.launch.py')
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_drivers')),
    )

    # 2. Roboterarm (Safety MPC + Arm Handler); servo_node nur mitstarten, wenn drivers aus sind
    arm_launch_args = {
        'config_file': LaunchConfiguration('config_file'),
        'launch_driver': PythonExpression([
            "'false' if '", LaunchConfiguration('launch_drivers'), "'.lower() == 'true' else 'true'"
        ]),
        'log_level': LaunchConfiguration('log_level'),
    }
    if default_safety_config:
        arm_launch_args['safety_config_file'] = LaunchConfiguration('safety_config_file')

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_launch_share, 'launch', 'arm_mpc.launch.py')
        ),
        launch_arguments=arm_launch_args.items(),
        condition=IfCondition(LaunchConfiguration('launch_arm')),
    )

    # 3. Optionaler Kalibrierungsservice
    kalibration_node = Node(
        package='picar_kalibration',
        executable='kalibration_service_node',
        name='kalibration_service_node',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
        condition=IfCondition(LaunchConfiguration('launch_kalibration')),
    )

    return LaunchDescription([
        config_arg,
        safety_config_arg,
        log_level_arg,
        launch_drivers_arg,
        launch_arm_arg,
        launch_kalibration_arg,
        drivers_launch,
        arm_launch,
        kalibration_node,
    ])
