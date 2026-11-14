#!/usr/bin/env python3
"""
arm_mpc.launch.py - Startet den Acados MPC Safety Filter & C++ Arm Handler Node
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        default_config = os.path.join(
            get_package_share_directory("picar_drivers"),
            "config",
            "picar_params.yaml",
        )
    except PackageNotFoundError:
        default_config = ""

    try:
        default_safety_config = os.path.join(
            get_package_share_directory("picar_arm_safety"),
            "config",
            "safety_params.yaml",
        )
        if not os.path.isfile(default_safety_config):
            default_safety_config = ""
    except PackageNotFoundError:
        default_safety_config = ""

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Pfad zur YAML-Konfiguration (z. B. Startposition des Roboterarms)"
    )

    safety_config_arg = DeclareLaunchArgument(
        "safety_config_file",
        default_value=default_safety_config,
        description="Pfad zur Safety-MPC YAML-Konfiguration fuer picar_arm_safety_node"
    )

    launch_driver_arg = DeclareLaunchArgument(
        "launch_driver",
        default_value="false",
        description="Falls true, wird auch servo_node aus picar_drivers gestartet"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log-Level (debug, info, warn, error)"
    )

    config_file = LaunchConfiguration("config_file")
    safety_config_file = LaunchConfiguration("safety_config_file")
    node_params = [config_file] if default_config else []
    safety_params = [safety_config_file] if default_safety_config else []

    safety_solver_node = Node(
        package="picar_arm_safety",
        executable="picar_arm_safety_node",
        name="picar_arm_safety_node",
        output="screen",
        parameters=safety_params,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    )

    arm_handler_node = Node(
        package="picar_arm_handler",
        executable="arm_handler_node",
        name="arm_handler_node",
        output="screen",
        parameters=node_params,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    )

    servo_driver_node = Node(
        package="picar_drivers",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=node_params,
        condition=IfCondition(LaunchConfiguration("launch_driver"))
    )

    return LaunchDescription([
        config_arg,
        safety_config_arg,
        launch_driver_arg,
        log_level_arg,
        safety_solver_node,
        arm_handler_node,
        servo_driver_node
    ])
