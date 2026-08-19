"""
Project Zero — SAFiR Lab
Top-Level System Launch File (One Command Bringup)

Launches the complete autonomous pipeline:
  1. Gazebo Harmonic Simulation (if sim:=true)
  2. Automatic Simulation Clock Unpause (if sim:=true)
  3. 3D LiDAR Pointcloud -> 2D LaserScan Bridge (only in simulation)
  4. AMCL Localization & Map Server
  5. Nav2 Autonomy Stack (MPPI Controller, Smac/NavFn Planner, BT Navigator)
  6. YOLOv8 Real-time Vision Detector Node (auto-namespaced)
  7. RViz2 Visualizer (Optional: disabled when rviz:=false)
  8. Autonomous Multi-Waypoint Mission Orchestrator
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def launch_setup(context, *args, **kwargs):
    pkg_jackal_mission = get_package_share_directory('jackal_mission')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    # Read launch configuration values
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    use_sim_time_in = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    use_sim_time = (sim and use_sim_time_in)
    use_sim_time_str = 'true' if use_sim_time else 'false'
    
    setup_path_val = LaunchConfiguration('setup_path').perform(context)
    map_path = LaunchConfiguration('map').perform(context)
    waypoints_file = LaunchConfiguration('waypoints').perform(context)
    run_mission = LaunchConfiguration('mission').perform(context).lower() == 'true'

    # Support both rviz and use_rviz flags
    rviz_val = LaunchConfiguration('rviz').perform(context).lower()
    use_rviz_val = LaunchConfiguration('use_rviz').perform(context).lower()
    show_rviz = (rviz_val == 'true') and (use_rviz_val == 'true')

    # Detect robot namespace dynamically from robot.yaml (e.g. j100_0751 on real hardware, j100_0000 in sim)
    namespace_val = LaunchConfiguration('namespace').perform(context)
    robot_yaml_path = os.path.join(setup_path_val, 'robot.yaml')
    if os.path.isfile(robot_yaml_path):
        try:
            with open(robot_yaml_path, 'r') as f:
                robot_data = yaml.safe_load(f)
                if robot_data and 'serial_number' in robot_data:
                    namespace_val = robot_data['serial_number'].replace('-', '_')
                elif robot_data and 'system' in robot_data and 'namespace' in robot_data['system']:
                    namespace_val = robot_data['system']['namespace']
        except Exception:
            pass

    default_rviz_config = os.path.join(pkg_jackal_vision, 'config', 'mission_viz.rviz')
    actions = []

    # ---------------- 1. Simulation (Only if sim:=true) ----------------
    if sim:
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
            ),
            launch_arguments={'rviz': 'false'}.items(),
        )
        actions.append(sim_launch)

        # Unpause simulation clock after Gazebo starts
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
        actions.append(unpause_clock)

        # In simulation, bridge 3D LiDAR pointcloud to 2D LaserScan
        pointcloud_to_laserscan_node = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            namespace=namespace_val,
            remappings=[
                ('cloud_in', f'/{namespace_val}/sensors/lidar3d_0/points'),
                ('scan', f'/{namespace_val}/sensors/lidar2d_0/scan'),
                ('/tf', f'/{namespace_val}/tf'),
                ('/tf_static', f'/{namespace_val}/tf_static'),
            ],
            parameters=[{
                'target_frame': 'lidar3d_0_laser',
                'transform_tolerance': 0.5,
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
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        )
        actions.append(pointcloud_to_laserscan_node)

    scan_topic_val = f'/{namespace_val}/sensors/lidar2d_0/scan' if sim else f'/{namespace_val}/sensors/lidar3d_0/scan'

    # ---------------- 2. Localization (AMCL + Map Server) ----------------
    localization_delay = 6.0 if sim else 1.0
    localization_launch = TimerAction(
        period=localization_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time_str,
                    'setup_path': setup_path_val,
                    'map': map_path,
                    'scan_topic': scan_topic_val,
                }.items(),
            )
        ],
    )
    actions.append(localization_launch)

    # ---------------- 3. Nav2 (Planners, Controllers, BT) ----------------
    nav2_delay = 7.0 if sim else 2.0
    nav2_launch = TimerAction(
        period=nav2_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time_str,
                    'setup_path': setup_path_val,
                    'scan_topic': scan_topic_val,
                }.items(),
            )
        ],
    )
    actions.append(nav2_launch)

    # ---------------- 4. YOLOv8 Vision Pipeline ----------------
    vision_delay = 8.0 if sim else 3.0
    vision_launch = TimerAction(
        period=vision_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_jackal_vision, 'launch', 'vision.launch.py'])
                ),
                launch_arguments={
                    'namespace': namespace_val,
                    'use_sim_time': use_sim_time_str,
                }.items(),
            )
        ],
    )
    actions.append(vision_launch)

    # ---------------- 5. RViz2 Visualizer (Only if enabled) ----------------
    if show_rviz:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', f'/{namespace_val}/tf'),
                ('/tf_static', f'/{namespace_val}/tf_static'),
                ('/robot_description', f'/{namespace_val}/robot_description'),
                ('map', f'/{namespace_val}/map'),
                ('initialpose', f'/{namespace_val}/initialpose'),
                ('goal_pose', f'/{namespace_val}/goal_pose'),
            ],
            output='screen',
        )
        actions.append(rviz_node)

    # ---------------- 6. Mission Orchestrator Node (If mission:=true) ----------------
    if run_mission:
        mission_delay = 18.0 if sim else 6.0
        mission_node = TimerAction(
            period=mission_delay,
            actions=[
                Node(
                    package='jackal_mission',
                    executable='mission_node',
                    name='mission_orchestrator',
                    namespace=namespace_val,
                    parameters=[
                        os.path.join(pkg_jackal_mission, 'config', 'mission_params.yaml'),
                        {'use_sim_time': use_sim_time},
                        {'waypoints_file': waypoints_file},
                    ],
                    output='screen',
                )
            ],
        )
        actions.append(mission_node)

    return actions


def generate_launch_description():
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    default_map = os.path.join(
        os.path.expanduser('~'), 'ali', 'Project-Zero', 'maps', 'warehouse_map.yaml'
    )
    if not os.path.exists(default_map):
        default_map = os.path.join(
            pkg_clearpath_nav2_demos, 'maps', 'warehouse.yaml'
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
        description='Launch RViz2 visualizer (alias for use_rviz)',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualizer',
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
        default_value='j100_0751',
        description='Robot namespace (auto-detected from robot.yaml if available)',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
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

    return LaunchDescription([
        sim_arg,
        rviz_arg,
        use_rviz_arg,
        waypoints_arg,
        mission_arg,
        namespace_arg,
        use_sim_time_arg,
        map_arg,
        setup_path_arg,
        OpaqueFunction(function=launch_setup),
    ])
