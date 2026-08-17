#!/usr/bin/env python3
"""
Slope Traversability & Inclinometer Safety Monitor Node
Calculates real-time Pitch, Roll, and Total Terrain Slope inclination from IMU data.
Publishes 3D Inclinometer RViz Markers and slope safety telemetry.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_euler(x, y, z, w):
    """Convert a quaternion into euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class SlopeInclinometerNode(Node):
    def __init__(self):
        super().__init__('slope_inclinometer_node')

        # Parameters
        self.declare_parameter('caution_slope_deg', 15.0)
        self.declare_parameter('danger_slope_deg', 25.0)
        self.declare_parameter('robot_frame', 'base_link')

        self.caution_slope = self.get_parameter('caution_slope_deg').value
        self.danger_slope = self.get_parameter('danger_slope_deg').value
        self.robot_frame = self.get_parameter('robot_frame').value

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.telemetry_pub = self.create_publisher(
            Vector3Stamped, 'slope_safety/telemetry', 10
        )
        self.status_pub = self.create_publisher(
            String, 'slope_safety/status', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, 'slope_safety/markers', 10
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, 'sensors/imu_0/data', self.imu_callback, sensor_qos
        )

        self.get_logger().info('Slope Traversability & Inclinometer Safety Monitor initialized.')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        roll_rad, pitch_rad, yaw_rad = quaternion_to_euler(q.x, q.y, q.z, q.w)

        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)

        # Total terrain slope angle relative to gravity vector
        total_slope_deg = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll_rad) * math.cos(pitch_rad)))))

        # Determine safety status
        if total_slope_deg >= self.danger_slope:
            status = 'DANGER (TIPPING RISK)'
            color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # Red
        elif total_slope_deg >= self.caution_slope:
            status = 'CAUTION (STEEP SLOPE)'
            color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)  # Yellow
        else:
            status = 'SAFE (TRAVERSABLE)'
            color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.9)  # Green

        # 1. Publish Telemetry
        vec_msg = Vector3Stamped()
        vec_msg.header = msg.header
        vec_msg.header.frame_id = self.robot_frame
        vec_msg.vector.x = roll_deg
        vec_msg.vector.y = pitch_deg
        vec_msg.vector.z = total_slope_deg
        self.telemetry_pub.publish(vec_msg)

        # 2. Publish Status
        status_msg = String()
        status_msg.data = f'{status} | Slope: {total_slope_deg:.1f}° (Pitch: {pitch_deg:.1f}°, Roll: {roll_deg:.1f}°)'
        self.status_pub.publish(status_msg)

        # 3. Publish 3D RViz Markers
        self.publish_markers(msg.header, roll_deg, pitch_deg, total_slope_deg, status, color)

    def publish_markers(self, header, roll_deg, pitch_deg, total_slope_deg, status, color):
        markers = MarkerArray()

        # Marker 1: Floating 3D HUD Text above Jackal
        text_marker = Marker()
        text_marker.header.stamp = header.stamp
        text_marker.header.frame_id = self.robot_frame
        text_marker.ns = 'slope_hud'
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.65  # Floating 65cm above chassis
        text_marker.scale.z = 0.12  # Text height
        text_marker.color = color
        text_marker.text = f'SLOPE: {total_slope_deg:.1f}° [{status}]\nPitch: {pitch_deg:.1f}° | Roll: {roll_deg:.1f}°'
        markers.markers.append(text_marker)

        # Marker 2: Inclinometer Horizon Ring on Chassis
        ring_marker = Marker()
        ring_marker.header.stamp = header.stamp
        ring_marker.header.frame_id = self.robot_frame
        ring_marker.ns = 'slope_ring'
        ring_marker.id = 1
        ring_marker.type = Marker.CYLINDER
        ring_marker.action = Marker.ADD
        ring_marker.pose.position.x = 0.0
        ring_marker.pose.position.y = 0.0
        ring_marker.pose.position.z = 0.2
        ring_marker.scale.x = 0.6
        ring_marker.scale.y = 0.6
        ring_marker.scale.z = 0.02
        ring_marker.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.4)
        markers.markers.append(ring_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SlopeInclinometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
