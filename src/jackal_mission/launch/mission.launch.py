"""
Project Zero — SAFiR Lab
Top-level mission launch file.

Brings up the FULL autonomy pipeline in one command:

  1. pointcloud_to_laserscan   (VLP-16 3D → 2D scan bridge)
  2. Nav2  (localization with AMCL + navigation)
  3. YOLO vision pipeline
  4. Mission orchestrator  (waypoint sequencer + detection collector)

Usage (simulation — make sure clearpath_gz simulation is already running):
  ros2 launch jackal_mission mission.launch.py \
      use_sim_time:=true \
      map:=/path/to/warehouse_map.yaml

Usage (real robot):
  ros2 launch jackal_mission mission.launch.py \
      map:=/path/to/warehouse_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Package directories ----
    pkg_jackal_mission = get_package_share_directory('jackal_mission')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    # ---- Declare arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        choices=['true', 'false'],
        description='Use simulated clock',
    )

    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Full path to the map YAML file (e.g. warehouse_map.yaml)',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='j100_0000',
        description='Robot namespace (must match robot.yaml)',
    )

    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=os.path.join(os.path.expanduser('~'), 'clearpath', ''),
        description='Clearpath config directory containing robot.yaml',
    )

    # ---- 1. Pointcloud-to-laserscan bridge ----
    # This launch file lives in the workspace root /launch directory.
    # We find it relative to this package or use the workspace path.
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace=LaunchConfiguration('namespace'),
        remappings=[
            ('cloud_in', [
                '/', LaunchConfiguration('namespace'),
                '/sensors/lidar3d_0/points',
            ]),
            ('scan', [
                '/', LaunchConfiguration('namespace'),
                '/sensors/lidar2d_0/scan',
            ]),
            ('/tf', [
                '/', LaunchConfiguration('namespace'), '/tf',
            ]),
            ('/tf_static', [
                '/', LaunchConfiguration('namespace'), '/tf_static',
            ]),
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

    # ---- 2. Nav2 (localization + navigation) ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': LaunchConfiguration('setup_path'),
        }.items(),
    )

    # ---- 3. Localization (AMCL + map_server) ----
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'setup_path': LaunchConfiguration('setup_path'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    # ---- 4. Vision pipeline (delayed start to let Nav2 settle) ----
    vision_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        pkg_jackal_vision, 'launch', 'vision.launch.py',
                    ])
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            ),
        ],
    )

    # ---- 5. Mission node (delayed to let localization converge) ----
    mission_params = os.path.join(
        pkg_jackal_mission, 'config', 'mission_params.yaml'
    )
    waypoints_file = os.path.join(
        pkg_jackal_mission, 'config', 'waypoints.yaml'
    )

    mission_node = TimerAction(
        period=15.0,  # Give Nav2 + AMCL time to initialise
        actions=[
            Node(
                package='jackal_mission',
                executable='mission_node',
                name='mission_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    mission_params,
                    {
                        'waypoints_file': waypoints_file,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    },
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        namespace_arg,
        setup_path_arg,
        pointcloud_to_laserscan_node,
        localization_launch,
        nav2_launch,
        vision_launch,
        mission_node,
    ])
