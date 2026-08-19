"""
Project Zero — SAFiR Lab
Launch file for RViz2 with pre-configured mission visualizer for both physical robot and simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    config_override = LaunchConfiguration('rviz_config').perform(context)
    if len(config_override) > 0 and os.path.exists(config_override):
        rviz_config_path = config_override
    else:
        if use_sim_time:
            rviz_config_path = os.path.join(pkg_jackal_vision, 'config', 'mission_viz.rviz')
        else:
            rviz_config_path = os.path.join(pkg_jackal_vision, 'config', 'physical_viz.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', f'/{namespace}/tf'),
            ('/tf_static', f'/{namespace}/tf_static'),
        ],
        output='screen',
    )

    return [rviz_node]


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0751',
        description='Robot namespace (must match robot.yaml)',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Path to custom RViz configuration file',
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        rviz_config_arg,
        OpaqueFunction(function=launch_setup),
    ])
