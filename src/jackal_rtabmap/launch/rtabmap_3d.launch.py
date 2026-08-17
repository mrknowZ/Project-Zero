import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_jackal_rtabmap = get_package_share_directory('jackal_rtabmap')
    default_config_path = os.path.join(pkg_jackal_rtabmap, 'config', 'rtabmap_3d.yaml')
    default_rviz_config = os.path.join(pkg_jackal_rtabmap, 'config', 'rtabmap_3d_viz.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0000',
        description='Robot namespace',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with 3D elevation and OctoMap visualization',
    )

    # RTAB-Map 3D Node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            default_config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('scan_cloud', ['/', LaunchConfiguration('namespace'), '/sensors/lidar3d_0/points']),
            ('rgb/image', ['/', LaunchConfiguration('namespace'), '/sensors/camera_0/color/image']),
            ('rgb/camera_info', ['/', LaunchConfiguration('namespace'), '/sensors/camera_0/color/camera_info']),
            ('odom', ['/', LaunchConfiguration('namespace'), '/platform/odom']),
            ('map', ['/', LaunchConfiguration('namespace'), '/map']),
            ('grid_map', ['/', LaunchConfiguration('namespace'), '/grid_map']),
            ('cloud_map', ['/', LaunchConfiguration('namespace'), '/cloud_map']),
            ('cloud_ground', ['/', LaunchConfiguration('namespace'), '/cloud_ground']),
            ('cloud_obstacles', ['/', LaunchConfiguration('namespace'), '/cloud_obstacles']),
            ('octomap_grid', ['/', LaunchConfiguration('namespace'), '/octomap_grid']),
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
        ],
        arguments=['-d'],
        output='screen',
    )

    # 3D RViz2 Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rtabmap_rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
            ('/robot_description', ['/', LaunchConfiguration('namespace'), '/robot_description']),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        use_rviz_arg,
        rtabmap_node,
        rviz_node,
    ])
