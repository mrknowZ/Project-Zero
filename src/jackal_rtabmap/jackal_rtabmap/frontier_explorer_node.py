#!/usr/bin/env python3
"""
Autonomous Frontier Exploration Node for Clearpath Jackal J100
Detects map boundaries (frontiers between known and unknown cells),
clusters them, selects the optimal frontier, and dispatches Nav2 goals
until 100% environment exploration is achieved.
"""

import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
import tf2_ros


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        # Parameters
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_rate_hz', 0.5)
        self.declare_parameter('gain_weight', 1.5)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.gain_weight = self.get_parameter('gain_weight').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.is_exploring = self.get_parameter('auto_start').value

        # State
        self.current_map = None
        self.robot_pose = None
        self.active_goal_handle = None
        self.is_navigating = False
        self.blacklist = []  # List of unreachable (x, y) coordinates
        self.blacklist_threshold = 0.8  # meters

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.status_pub = self.create_publisher(String, 'exploration/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'exploration/frontiers', 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos
        )

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Main Exploration Loop Timer
        self.loop_timer = self.create_timer(2.0, self.exploration_cycle)

        self.get_logger().info('Autonomous Frontier Exploration Node initialized.')

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def find_frontiers(self, grid_data, width, height, resolution, origin_x, origin_y):
        """Find boundary cells between Free (0) and Unknown (-1)."""
        grid = np.array(grid_data, dtype=np.int8).reshape((height, width))

        # Boolean masks
        is_free = (grid == 0)
        is_unknown = (grid == -1)

        # Neighbor kernel (8-connectivity)
        frontiers = []
        free_indices = np.argwhere(is_free)

        for r, c in free_indices:
            # Check 8 neighbors for unknown cells
            r_min = max(0, r - 1)
            r_max = min(height, r + 2)
            c_min = max(0, c - 1)
            c_max = min(width, c + 2)

            neighborhood = is_unknown[r_min:r_max, c_min:c_max]
            if np.any(neighborhood):
                wx = origin_x + (c + 0.5) * resolution
                wy = origin_y + (r + 0.5) * resolution
                frontiers.append((wx, wy))

        return frontiers

    def cluster_frontiers(self, points, cluster_radius=0.6):
        """Simple distance-based clustering for frontier points."""
        clusters = []
        visited = set()

        for i, p in enumerate(points):
            if i in visited:
                continue
            cluster = [p]
            visited.add(i)

            for j, q in enumerate(points):
                if j not in visited:
                    dist = math.hypot(p[0] - q[0], p[1] - q[1])
                    if dist <= cluster_radius:
                        cluster.append(q)
                        visited.add(j)

            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_threshold:
                return True
        return False

    def exploration_cycle(self):
        if not self.is_exploring:
            return

        if self.current_map is None:
            self.publish_status('WAITING FOR MAP')
            return

        self.robot_pose = self.get_robot_pose()
        if self.robot_pose is None:
            self.publish_status('WAITING FOR ROBOT POSE')
            return

        # If currently navigating to a target, let it proceed
        if self.is_navigating:
            return

        # 1. Detect Frontiers
        info = self.current_map.info
        raw_frontiers = self.find_frontiers(
            self.current_map.data,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y
        )

        if not raw_frontiers:
            self.publish_status('NO FRONTIERS DETECTED - MAP COVERED!')
            return

        # 2. Cluster Frontiers
        clusters = self.cluster_frontiers(raw_frontiers)
        if not clusters:
            self.publish_status('FRONTIERS TOO SMALL - MAP 100% COMPLETE!')
            return

        # 3. Score & Select Best Frontier
        rx, ry = self.robot_pose
        best_target = None
        best_score = float('inf')
        frontier_centroids = []

        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            frontier_centroids.append((cx, cy, len(cluster)))

            if self.is_blacklisted(cx, cy):
                continue

            dist = math.hypot(rx - cx, ry - cy)
            # Score: shorter distance is better, larger cluster size is better
            score = dist - self.gain_weight * math.log(max(1, len(cluster)))

            if score < best_score:
                best_score = score
                best_target = (cx, cy)

        # 4. Publish 3D Frontier Markers to RViz
        self.publish_markers(frontier_centroids, best_target)

        # 5. Dispatch Goal to Nav2
        if best_target:
            self.publish_status(
                f'EXPLORING | Frontiers: {len(clusters)} | Target: [{best_target[0]:.2f}, {best_target[1]:.2f}]'
            )
            self.send_nav_goal(best_target[0], best_target[1])
        else:
            self.publish_status('EXPLORATION COMPLETE (All reachable frontiers explored)')

    def send_nav_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 Action Server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Face towards the frontier direction
        rx, ry = self.robot_pose
        yaw = math.atan2(y - ry, x - rx)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.is_navigating = True
        self.get_logger().info(f'Dispatching exploration goal: ({x:.2f}, {y:.2f})')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, (x, y))
        )

    def goal_response_callback(self, future, target):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal {target} was rejected by Nav2')
            self.blacklist.append(target)
            self.is_navigating = False
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda res_future: self.goal_result_callback(res_future, target)
        )

    def goal_result_callback(self, future, target):
        self.is_navigating = False
        status = future.result().status
        # Status 4 = STATUS_SUCCEEDED
        if status == 4:
            self.get_logger().info(f'Reached frontier goal {target} successfully!')
        else:
            self.get_logger().warn(f'Goal {target} aborted/canceled. Adding to blacklist.')
            self.blacklist.append(target)

    def nav_feedback_callback(self, feedback_msg):
        pass

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_markers(self, centroids, active_target):
        markers = MarkerArray()

        # Marker 1: All Candidate Frontiers (Cyan Spheres)
        points_marker = Marker()
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.header.frame_id = self.map_frame
        points_marker.ns = 'candidate_frontiers'
        points_marker.id = 1
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.25
        points_marker.scale.y = 0.25
        points_marker.scale.z = 0.25
        points_marker.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.8)  # Cyan

        for cx, cy, _ in centroids:
            p = Point()
            p.x = cx
            p.y = cy
            p.z = 0.15
            points_marker.points.append(p)
        markers.markers.append(points_marker)

        # Marker 2: Active Selected Target (Glowing Gold Marker)
        if active_target:
            target_marker = Marker()
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.header.frame_id = self.map_frame
            target_marker.ns = 'active_frontier_target'
            target_marker.id = 2
            target_marker.type = Marker.CYLINDER
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = active_target[0]
            target_marker.pose.position.y = active_target[1]
            target_marker.pose.position.z = 0.3
            target_marker.scale.x = 0.45
            target_marker.scale.y = 0.45
            target_marker.scale.z = 0.6
            target_marker.color = ColorRGBA(r=1.0, g=0.84, b=0.0, a=0.9)  # Gold
            markers.markers.append(target_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
