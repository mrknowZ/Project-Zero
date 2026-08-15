"""
Project Zero — SAFiR Lab
Launch file for RViz2 with pre-configured mission visualizer.
Properly namespaces /tf, /tf_static, and /robot_description for full 3D robot model rendering.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    default_rviz_config = os.path.join(
        pkg_jackal_vision, 'config', 'mission_viz.rviz'
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

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz configuration file',
    )

    rviz_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
            output='screen',
        )
    ])

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        rviz_config_arg,
        rviz_group,
    ])
