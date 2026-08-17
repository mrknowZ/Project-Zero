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


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_jackal_vision = get_package_share_directory('jackal_vision')
    pkg_jackal_rtabmap = get_package_share_directory('jackal_rtabmap')

    # Arguments
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch Gazebo Harmonic simulation',
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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch 3D RViz2 visualizer',
    )

    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='Launch YOLOv8 object detector',
    )

    # 1. Gazebo Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={'rviz': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # 2. Clock Unpause Timer
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

    # 3. 3D to 2D LiDAR Bridge
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
        condition=IfCondition(LaunchConfiguration('use_yolo')),
    )

    # 5. RTAB-Map 3D SLAM & 3D Visualizer
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_jackal_rtabmap, 'launch', 'rtabmap_3d.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'delete_db_on_start': 'true',
        }.items(),
    )
    delayed_rtabmap = TimerAction(period=7.0, actions=[rtabmap_launch])

    return LaunchDescription([
        sim_arg,
        use_sim_time_arg,
        namespace_arg,
        use_rviz_arg,
        use_yolo_arg,
        sim_launch,
        delayed_unpause,
        bridge_launch,
        yolo_launch,
        delayed_rtabmap,
    ])
