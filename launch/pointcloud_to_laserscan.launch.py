"""

Project Zero — SAFiR Lab

Converts the Velodyne VLP-16's 3D PointCloud2 into a 2D LaserScan, which
SLAM Toolbox / AMCL / Nav2 (via clearpath_nav2_demos configs) expect on
the 'lidar2d_0' topic. The Clearpath jazzy simulator stack for this robot
only declares a `lidar3d` sensor in robot.yaml, so this conversion step
does NOT happen automatically — this launch file fills that gap.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0000',
        description='Robot namespace, must match robot.yaml system.ros2.namespace'
    )

    namespace = LaunchConfiguration('namespace')

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace=namespace,
        remappings=[
            ('cloud_in', ['/', namespace, '/sensors/lidar3d_0/points']),
            ('scan', ['/', namespace, '/sensors/lidar2d_0/scan']),
            ('/tf', ['/', namespace, '/tf']),
            ('/tf_static', ['/', namespace, '/tf_static']),
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
        }],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        pointcloud_to_laserscan_node,
    ])
