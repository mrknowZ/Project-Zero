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
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    pkg_jackal_rtabmap = get_package_share_directory('jackal_rtabmap')

    default_rviz_config = os.path.join(pkg_jackal_rtabmap, 'config', 'rtabmap_3d_viz.rviz')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='World environment (e.g. warehouse, depot, or custom outdoor world)',
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

    # 1. Gazebo Simulation with dynamic world
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

    # 3. 3D LiDAR Bridge (General 35m range)
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.expanduser('~'),
                'ali', 'Project-Zero', 'launch', 'pointcloud_to_laserscan.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    # 4. YOLOv8 Detector
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_jackal_vision, 'launch', 'vision.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    # 5. Slope & Inclinometer Safety Monitor (Terrain Tilt & Tipping Protection)
    slope_node = Node(
        package='jackal_rtabmap',
        executable='slope_inclinometer_node',
        name='slope_inclinometer_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'caution_slope_deg': 15.0,
                'danger_slope_deg': 25.0,
            }
        ],
        output='screen',
    )

    # 6. 3D Orbit RViz Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_3d_terrain',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', ['/', LaunchConfiguration('namespace'), '/tf']),
            ('/tf_static', ['/', LaunchConfiguration('namespace'), '/tf_static']),
            ('/robot_description', ['/', LaunchConfiguration('namespace'), '/robot_description']),
        ],
        output='screen',
    )
    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        namespace_arg,
        sim_launch,
        delayed_unpause,
        bridge_launch,
        yolo_launch,
        slope_node,
        delayed_rviz,
    ])
