#!/usr/bin/env python3
"""
arm_mpc.launch.py - Startet den Acados MPC Safety Filter & C++ Arm Handler Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_driver_arg = DeclareLaunchArgument(
        "launch_driver",
        default_value="false",
        description="Falls true, wird auch motor_node aus picar_drivers gestartet"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log-Level (debug, info, warn, error)"
    )

    safety_solver_node = Node(
        package="picar_arm_safety",
        executable="picar_arm_safety_node",
        name="picar_arm_safety_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    )

    arm_handler_node = Node(
        package="picar_arm_handler",
        executable="arm_handler_node",
        name="arm_handler_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    )

    motor_driver_node = Node(
        package="picar_drivers",
        executable="motor_node",
        name="motor_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_driver"))
    )

    return LaunchDescription([
        launch_driver_arg,
        log_level_arg,
        safety_solver_node,
        arm_handler_node,
        motor_driver_node
    ])
