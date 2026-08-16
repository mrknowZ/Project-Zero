"""
Project Zero — SAFiR Lab
Top-Level System Launch File (One Command Bringup)

Launches the complete autonomous pipeline:
  1. Gazebo Harmonic Simulation (Clearpath Jackal in Warehouse)
  2. Automatic Simulation Clock Unpause
  3. 3D LiDAR Pointcloud -> 2D LaserScan Bridge
  4. AMCL Localization & Map Server
  5. Nav2 Autonomy Stack (MPPI Controller, Smac/NavFn Planner, BT Navigator)
  6. YOLOv8 Real-time Vision Detector Node
  7. RViz2 Visualizer (Pre-configured with Map, RobotModel, Costmaps, YOLO Feed)
  8. Dedicated Camera View Window (rqt_image_view on YOLO Detections)
  9. Autonomous Multi-Waypoint Mission Orchestrator
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_jackal_mission = get_package_share_directory('jackal_mission')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    default_map = os.path.join(
        os.path.expanduser('~'), 'ali', 'Project-Zero', 'maps', 'warehouse_map.yaml'
    )
    if not os.path.exists(default_map):
        default_map = os.path.join(
            pkg_clearpath_nav2_demos, 'maps', 'warehouse.yaml'
        )

    default_rviz_config = os.path.join(
        pkg_jackal_vision, 'config', 'mission_viz.rviz'
    )

    # ---------------- Arguments ----------------
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch Gazebo Harmonic simulation',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 visualizer',
    )

    camera_view_arg = DeclareLaunchArgument(
        'camera_view',
        default_value='true',
        description='Launch dedicated rqt_image_view window for real-time YOLO camera detections',
    )

    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='',
        description='Path to custom waypoints YAML file (e.g. for real robot tests)',
    )

    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='true',
        description='Run autonomous waypoint mission orchestrator',
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

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to occupancy grid map YAML',
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

    # ---------------- 3. Localization (AMCL + Map Server) ----------------
    localization_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'setup_path': LaunchConfiguration('setup_path'),
                    'map': LaunchConfiguration('map'),
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
            )
        ],
    )

    # ---------------- 5. YOLOv8 Vision Pipeline ----------------
    vision_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_jackal_vision, 'launch', 'vision.launch.py'])
                ),
                launch_arguments={
                    'namespace': LaunchConfiguration('namespace'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ],
    )

    # ---------------- 6. RViz2 Visualizer ----------------
    rviz_group = TimerAction(
        period=9.0,
        actions=[
            GroupAction([
                PushRosNamespace(LaunchConfiguration('namespace')),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', default_rviz_config],
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static'),
                    ],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('rviz')),
                )
            ])
        ]
    )

    # ---------------- 7. Camera View Window (rqt_image_view) ----------------
    camera_view_action = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='yolo_camera_viewer',
                arguments=['/j100_0000/yolo_detector/detections_image'],
                condition=IfCondition(LaunchConfiguration('camera_view')),
            )
        ],
    )

    # ---------------- 8. Mission Orchestrator Node ----------------
    mission_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='jackal_mission',
                executable='mission_node',
                name='mission_orchestrator',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    os.path.join(pkg_jackal_mission, 'config', 'mission_params.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'waypoints_file': LaunchConfiguration('waypoints')},
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('mission')),
            )
        ],
    )

    return LaunchDescription([
        sim_arg,
        rviz_arg,
        camera_view_arg,
        waypoints_arg,
        mission_arg,
        namespace_arg,
        use_sim_time_arg,
        map_arg,
        setup_path_arg,
        sim_launch,
        unpause_clock,
        pointcloud_to_laserscan_node,
        localization_launch,
        nav2_launch,
        vision_launch,
        rviz_group,
        camera_view_action,
        mission_node,
    ])
