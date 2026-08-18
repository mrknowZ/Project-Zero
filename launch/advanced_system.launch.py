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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    pkg_jackal_rtabmap = get_package_share_directory('jackal_rtabmap')

    default_rviz_config = os.path.join(pkg_jackal_rtabmap, 'config', 'rtabmap_3d_viz.rviz')
    default_setup_path = os.path.join(os.path.expanduser('~'), 'clearpath')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='World environment name (e.g. warehouse, depot, outdoor)',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0000',
        description='Robot namespace (matches robot.yaml)',
    )

    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value=default_setup_path,
        description='Path to robot.yaml configuration folder',
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start_exploration',
        default_value='true',
        description='Automatically dispatch frontier exploration goals',
    )

    # 1. Gazebo Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={
            'rviz': 'false',
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # 2. Clock Unpause Timer (5s)
    unpause_cmd = ExecuteProcess(
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
    delayed_unpause = TimerAction(period=5.0, actions=[unpause_cmd])

    # 3. 3D LiDAR Bridge
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.expanduser('~'),
                'ali', 'Project-Zero', 'launch', 'pointcloud_to_laserscan.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    # 4. SLAM Toolbox Mapping (Clearpath Demos)
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

    # 5. Nav2 Autonomy Navigation
    nav2_launch = TimerAction(
        period=8.0,
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

    # 6. YOLOv8 Detector
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_jackal_vision, 'launch', 'vision.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    # 7. Slope & Inclinometer Safety Monitor
    slope_node = Node(
        package='jackal_rtabmap',
        executable='slope_inclinometer_node',
        name='slope_inclinometer_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # 8. Autonomous Frontier Exploration Node
    explorer_node = Node(
        package='jackal_rtabmap',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'auto_start': LaunchConfiguration('auto_start_exploration'),
                'min_frontier_size': 5,
                'gain_weight': 1.5,
            }
        ],
        remappings=[
            ('map', ['/', LaunchConfiguration('namespace'), '/map']),
            ('navigate_to_pose', ['/', LaunchConfiguration('namespace'), '/navigate_to_pose']),
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
        ],
        output='screen',
    )
    delayed_explorer = TimerAction(period=14.0, actions=[explorer_node])

    # 9. 3D Semantic Object Mapper Node
    semantic_mapper_node = Node(
        package='jackal_rtabmap',
        executable='semantic_object_mapper_node',
        name='semantic_object_mapper_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_frame': 'camera_0_link',
                'map_frame': 'map',
            }
        ],
        remappings=[
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
        ],
        output='screen',
    )

    # 10. Web & Tablet Telemetry Bridge Node (Port 8080)
    web_bridge_node = Node(
        package='jackal_rtabmap',
        executable='web_telemetry_bridge_node',
        name='web_telemetry_bridge_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    # 11. 3D Orbit RViz Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_exploration',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
            ('/robot_description', ['/', LaunchConfiguration('namespace'), '/robot_description']),
        ],
        output='screen',
    )
    delayed_rviz = TimerAction(period=7.0, actions=[rviz_node])

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        namespace_arg,
        setup_path_arg,
        auto_start_arg,
        sim_launch,
        delayed_unpause,
        bridge_launch,
        slam_launch,
        nav2_launch,
        yolo_launch,
        slope_node,
        delayed_explorer,
        semantic_mapper_node,
        web_bridge_node,
        delayed_rviz,
    ])
