import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('picar_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'picar.urdf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf,
            description='Pfad zur URDF-Datei des PiCar-Roboters'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Simulationszeit verwenden'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
