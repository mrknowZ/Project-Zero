"""
Project Zero — SAFiR Lab
Mapping Launch File (SLAM Toolbox + Physical Hardware / Gazebo + Teleop)

Directly launches SLAM Toolbox with exact hardware TF remappings so no laser scans are dropped.
"""

import os
from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')

    default_rviz_config = PathJoinSubstitution(
        [pkg_jackal_vision, 'config', 'physical_viz.rviz']
    )

    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'
    nav2 = LaunchConfiguration('nav2').perform(context).lower() == 'true'
    setup_path_val = LaunchConfiguration('setup_path').perform(context)

    # Resolve robot namespace dynamically
    namespace_val = LaunchConfiguration('namespace').perform(context)
    robot_yaml_path = os.path.join(setup_path_val, 'robot.yaml')
    if os.path.exists(robot_yaml_path):
        try:
            config = read_yaml(robot_yaml_path)
            clearpath_config = ClearpathConfig(config)
            namespace_val = clearpath_config.system.namespace
        except Exception:
            pass

    use_sim_time_val = sim
    scan_topic_val = f'/{namespace_val}/sensors/lidar2d_0/scan' if sim else f'/{namespace_val}/sensors/lidar3d_0/scan'

    actions = []

    # ---------------- 1. Simulation (Sim only) ----------------
    if sim:
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
            ),
            launch_arguments={'rviz': 'false'}.items(),
        )
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
                'use_sim_time': True,
            }],
            output='screen',
        )
        actions.extend([sim_launch, unpause_clock, pointcloud_to_laserscan_node])

    # ---------------- 2. SLAM Toolbox Node (Direct with Hardware TF Remappings) ----------------
    slam_config_file = PathJoinSubstitution([
        pkg_clearpath_nav2_demos, 'config', 'j100', 'slam.yaml'
    ])

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace_val,
        parameters=[
            slam_config_file,
            {
                'use_sim_time': use_sim_time_val,
                'scan_topic': scan_topic_val,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'map_name': f'/{namespace_val}/map',
                'transform_timeout': 0.5,
                'tf_buffer_duration': 30.0,
            }
        ],
        remappings=[
            ('/tf', f'/{namespace_val}/tf'),
            ('/tf_static', f'/{namespace_val}/tf_static'),
            ('scan', scan_topic_val),
            ('map', f'/{namespace_val}/map'),
            ('map_metadata', f'/{namespace_val}/map_metadata'),
        ],
        output='screen',
    )
    actions.append(slam_node)

    # ---------------- 3. Nav2 (Optional during mapping) ----------------
    if nav2:
        nav2_launch = TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])
                    ),
                    launch_arguments={
                        'use_sim_time': 'true' if sim else 'false',
                        'setup_path': setup_path_val,
                        'scan_topic': scan_topic_val,
                    }.items(),
                )
            ],
        )
        actions.append(nav2_launch)

    # ---------------- 4. RViz2 Visualizer ----------------
    if use_rviz:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config],
            parameters=[{'use_sim_time': sim}],
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

    return actions


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Launch Gazebo Harmonic simulation',
    )
    rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 visualizer',
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        description='Launch Nav2 stack for autonomous goal navigation while mapping',
    )
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
    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value='/etc/clearpath/',
        description='Clearpath config directory containing robot.yaml',
    )

    return LaunchDescription([
        sim_arg,
        rviz_arg,
        nav2_arg,
        namespace_arg,
        use_sim_time_arg,
        setup_path_arg,
        OpaqueFunction(function=launch_setup),
    ])
