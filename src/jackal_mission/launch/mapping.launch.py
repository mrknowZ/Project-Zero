"""
Project Zero — SAFiR Lab
Mapping Launch File (SLAM Toolbox + Gazebo + RViz2 + Nav2 / Teleop)

Launches:
  1. Gazebo Harmonic warehouse simulation
  2. Auto simulation clock unpause
  3. PointCloud-to-LaserScan 3D->2D converter
  4. SLAM Toolbox online mapping
  5. Nav2 autonomy stack (for setting goals while mapping)
  6. RViz2 visualizer with live map building display
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')

    default_rviz_config = PathJoinSubstitution(
        [pkg_jackal_vision, 'config', 'mission_viz.rviz']
    )

    # ---------------- Arguments ----------------
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch Gazebo Harmonic simulation',
    )

    rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualizer',
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch Nav2 stack for autonomous goal navigation while mapping',
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

    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=os.path.join(os.path.expanduser('~'), 'clearpath', ''),
        description='Clearpath config directory containing robot.yaml',
    )

    # ---------------- 1. Simulation ----------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={
            'rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # Automatically unpause the simulation clock 5 seconds after Gazebo starts
    unpause_clock = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service',
                    '-s', '/world/warehouse/control',
                    '--reqtype', 'ignition.msgs.WorldControl',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', 'pause: false',
                ],
                output='screen',
            )
        ],
    )

    # ---------------- 2. PointCloud-to-LaserScan Bridge ----------------
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace=LaunchConfiguration('namespace'),
        remappings=[
            ('cloud_in', ['/', LaunchConfiguration('namespace'), '/sensors/lidar3d_0/points']),
            ('scan', ['/', LaunchConfiguration('namespace'), '/sensors/lidar2d_0/scan']),
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
        ],
        parameters=[{
            'target_frame': 'lidar3d_0_laser',
            'transform_tolerance': 0.1,
            'min_height': -0.2,
            'max_height': 0.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0058,
            'scan_time': 0.3333,
            'range_min': 0.3,
            'range_max': 20.0,
            'use_inf': True,
            'concurrency_level': 1,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )

    # ---------------- 3. SLAM Toolbox Online Mapping ----------------
    slam_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'slam.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'setup_path': LaunchConfiguration('setup_path'),
                    'scan_topic': ['/', LaunchConfiguration('namespace'), '/sensors/lidar2d_0/scan'],
                }.items(),
            )
        ],
    )

    # ---------------- 4. Nav2 (Planners, Controllers, BT) ----------------
    nav2_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'setup_path': LaunchConfiguration('setup_path'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('nav2')),
            )
        ],
    )

    # ---------------- 5. RViz2 Visualizer ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
            ('/robot_description', ['/', LaunchConfiguration('namespace'), '/robot_description']),
            ('map', ['/', LaunchConfiguration('namespace'), '/map']),
            ('initialpose', ['/', LaunchConfiguration('namespace'), '/initialpose']),
            ('goal_pose', ['/', LaunchConfiguration('namespace'), '/goal_pose']),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        sim_arg,
        rviz_arg,
        nav2_arg,
        namespace_arg,
        use_sim_time_arg,
        setup_path_arg,
        sim_launch,
        unpause_clock,
        pointcloud_to_laserscan_node,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])
