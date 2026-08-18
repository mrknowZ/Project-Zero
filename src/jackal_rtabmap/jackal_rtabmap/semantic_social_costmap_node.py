#!/usr/bin/env python3
"""
Semantic Social & Object Exclusion Costmap Node for Clearpath Jackal J100
Fuses YOLOv8 3D semantic detections into a dynamic OccupancyGrid costmap layer:
  - People: Generates 1.2m radius Social Comfort Zones (smooth decaying cost).
  - Fragile / Obstacle Objects: Generates Keepout Exclusion Zones (lethal cost).
Publishes:
  - semantic_costmap/grid (nav_msgs/OccupancyGrid): Semantic costmap layer
  - semantic_costmap/social_zones (visualization_msgs/MarkerArray): 3D RViz markers
  - semantic_costmap/status (std_msgs/String): Real-time zone metrics
"""

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class SemanticSocialCostmapNode(Node):
    def __init__(self):
        super().__init__('semantic_social_costmap_node')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('social_zone_radius', 1.2)        # meters around people
        self.declare_parameter('keepout_zone_radius', 0.7)       # meters around fragile/hazard items
        self.declare_parameter('grid_resolution', 0.05)          # 5cm costmap cell resolution
        self.declare_parameter('grid_width_m', 20.0)             # 20m x 20m coverage
        self.declare_parameter('grid_height_m', 20.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.social_radius = self.get_parameter('social_zone_radius').value
        self.keepout_radius = self.get_parameter('keepout_zone_radius').value
        self.resolution = self.get_parameter('grid_resolution').value
        self.grid_w_m = self.get_parameter('grid_width_m').value
        self.grid_h_m = self.get_parameter('grid_height_m').value

        self.cells_w = int(self.grid_w_m / self.resolution)
        self.cells_h = int(self.grid_h_m / self.resolution)

        # Robot Pose
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Semantic Landmarks: list of dicts [{'id', 'label', 'x', 'y', 'z', ...}]
        self.semantic_objects = []

        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'semantic_costmap/grid', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'semantic_costmap/social_zones', 10)
        self.status_pub = self.create_publisher(String, 'semantic_costmap/status', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'platform/odom', self.odom_callback, 10
        )
        self.json_sub = self.create_subscription(
            String, 'semantic_map/objects_json', self.objects_callback, 10
        )

        # Periodic update loop (2 Hz)
        self.timer = self.create_timer(0.5, self.update_semantic_costmap)

        self.get_logger().info('Semantic Social & Exclusion Costmap Node initialized.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def objects_callback(self, msg: String):
        try:
            self.semantic_objects = json.loads(msg.data)
        except Exception as e:
            self.get_logger().debug(f'Error parsing semantic objects JSON: {e}')

    def update_semantic_costmap(self):
        # Center grid on current robot position
        origin_x = self.robot_x - (self.grid_w_m / 2.0)
        origin_y = self.robot_y - (self.grid_h_m / 2.0)

        # Initialize cost array with 0 (Free / No semantic penalty)
        cost_grid = np.zeros((self.cells_h, self.cells_w), dtype=np.int8)

        person_count = 0
        keepout_count = 0

        # Grid coordinate conversion helpers
        def world_to_grid(wx, wy):
            gx = int((wx - origin_x) / self.resolution)
            gy = int((wy - origin_y) / self.resolution)
            return gx, gy

        for obj in self.semantic_objects:
            label = obj.get('label', '').lower()
            ox = obj.get('x', 0.0)
            oy = obj.get('y', 0.0)

            # Check if object is within our local costmap window
            gx, gy = world_to_grid(ox, oy)
            if not (0 <= gx < self.cells_w and 0 <= gy < self.cells_h):
                continue

            if 'person' in label:
                person_count += 1
                # Social Comfort Zone: radius 1.2m
                rad_cells = int(self.social_radius / self.resolution)
                x_min = max(0, gx - rad_cells)
                x_max = min(self.cells_w, gx + rad_cells + 1)
                y_min = max(0, gy - rad_cells)
                y_max = min(self.cells_h, gy + rad_cells + 1)

                for cy in range(y_min, y_max):
                    for cx in range(x_min, x_max):
                        dist_m = math.hypot((cx - gx) * self.resolution, (cy - gy) * self.resolution)
                        if dist_m <= 0.4:
                            # Lethal inner body core
                            cost_grid[cy, cx] = max(cost_grid[cy, cx], 100)
                        elif dist_m <= self.social_radius:
                            # Soft decaying Gaussian social buffer: 80 down to 10
                            decay = (1.0 - (dist_m - 0.4) / (self.social_radius - 0.4))
                            social_cost = int(20 + 65 * decay)
                            cost_grid[cy, cx] = max(cost_grid[cy, cx], social_cost)

            else:
                keepout_count += 1
                # Keepout zone for other objects (chair, bottle, machinery, etc.)
                rad_cells = int(self.keepout_radius / self.resolution)
                x_min = max(0, gx - rad_cells)
                x_max = min(self.cells_w, gx + rad_cells + 1)
                y_min = max(0, gy - rad_cells)
                y_max = min(self.cells_h, gy + rad_cells + 1)

                for cy in range(y_min, y_max):
                    for cx in range(x_min, x_max):
                        dist_m = math.hypot((cx - gx) * self.resolution, (cy - gy) * self.resolution)
                        if dist_m <= self.keepout_radius:
                            cost_grid[cy, cx] = max(cost_grid[cy, cx], 90)

        # 1. Publish OccupancyGrid
        costmap_msg = OccupancyGrid()
        costmap_msg.header = Header()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = self.map_frame

        costmap_msg.info.resolution = float(self.resolution)
        costmap_msg.info.width = int(self.cells_w)
        costmap_msg.info.height = int(self.cells_h)
        costmap_msg.info.origin.position.x = float(origin_x)
        costmap_msg.info.origin.position.y = float(origin_y)
        costmap_msg.info.origin.position.z = 0.05
        costmap_msg.info.origin.orientation.w = 1.0

        costmap_msg.data = cost_grid.flatten().tolist()
        self.costmap_pub.publish(costmap_msg)

        # 2. Publish 3D Visual Zones in RViz
        self.publish_visual_zones()

        # 3. Publish Telemetry Status
        status_msg = String()
        status_msg.data = f"SEMANTIC_COSTMAP: ActivePeople={person_count} | KeepoutObjects={keepout_count} | TotalTracked={len(self.semantic_objects)}"
        self.status_pub.publish(status_msg)

    def publish_visual_zones(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Delete marker
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        markers.markers.append(del_m)

        for idx, obj in enumerate(self.semantic_objects):
            label = obj.get('label', '').lower()
            ox = obj.get('x', 0.0)
            oy = obj.get('y', 0.0)
            oz = max(0.1, obj.get('z', 0.0))

            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.map_frame
            marker.ns = 'semantic_zones'
            marker.id = idx + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = float(ox)
            marker.pose.position.y = float(oy)
            marker.pose.position.z = float(oz / 2.0)
            marker.pose.orientation.w = 1.0

            if 'person' in label:
                # Cyan Social Disc
                marker.scale.x = float(self.social_radius * 2.0)
                marker.scale.y = float(self.social_radius * 2.0)
                marker.scale.z = 0.12
                marker.color = ColorRGBA(r=0.0, g=0.9, b=0.9, a=0.45)
            else:
                # Amber Keepout Cylinder
                marker.scale.x = float(self.keepout_radius * 2.0)
                marker.scale.y = float(self.keepout_radius * 2.0)
                marker.scale.z = 0.25
                marker.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.55)

            markers.markers.append(marker)

        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSocialCostmapNode()
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
