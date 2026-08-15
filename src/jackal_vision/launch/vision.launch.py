"""
Project Zero — SAFiR Lab
Launch file for the YOLO vision pipeline.

Starts the yolo_detector_node with parameters from yolo_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_jackal_vision = get_package_share_directory('jackal_vision')

    default_params = os.path.join(
        pkg_jackal_vision, 'config', 'yolo_params.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to YOLO detector parameter file',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0000',
        description='Robot namespace (must match robot.yaml)',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    yolo_node = Node(
        package='jackal_vision',
        executable='yolo_detector_node',
        name='yolo_detector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_arg,
        namespace_arg,
        yolo_node,
    ])
