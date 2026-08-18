#!/usr/bin/env python3
"""
Slope & Terrain Traversability Costmap Node for Clearpath Jackal J100
Processes 3D LiDAR PointClouds into a 2.5D Elevation & Gradient Map.
Detects steep slopes, step drop-offs, and non-traversable terrain.
Publishes:
  - slope_costmap/grid (nav_msgs/OccupancyGrid): 2D traversability costmap
  - slope_costmap/hazard_markers (visualization_msgs/MarkerArray): 3D hazard markers
  - slope_costmap/status (std_msgs/String): Summary telemetry
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


def pointcloud2_to_xyz_array(cloud_msg: PointCloud2) -> np.ndarray:
    """Fast extraction of X, Y, Z from sensor_msgs/PointCloud2 without sensor_msgs_py."""
    if cloud_msg.width == 0 or cloud_msg.height == 0:
        return np.empty((0, 3), dtype=np.float32)

    fmt_dict = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
                5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}

    field_offsets = {}
    for f in cloud_msg.fields:
        if f.name in ['x', 'y', 'z']:
            field_offsets[f.name] = (f.offset, fmt_dict.get(f.datatype, np.float32))

    if len(field_offsets) < 3:
        return np.empty((0, 3), dtype=np.float32)

    point_step = cloud_msg.point_step
    n_points = cloud_msg.width * cloud_msg.height
    raw_data = np.frombuffer(cloud_msg.data, dtype=np.uint8)

    if len(raw_data) < n_points * point_step:
        return np.empty((0, 3), dtype=np.float32)

    # Reshape into (n_points, point_step)
    points_raw = raw_data[:n_points * point_step].reshape(n_points, point_step)

    # Extract x, y, z floats
    x_off = field_offsets['x'][0]
    y_off = field_offsets['y'][0]
    z_off = field_offsets['z'][0]

    x = np.frombuffer(points_raw[:, x_off:x_off + 4].copy(), dtype=np.float32)
    y = np.frombuffer(points_raw[:, y_off:y_off + 4].copy(), dtype=np.float32)
    z = np.frombuffer(points_raw[:, z_off:z_off + 4].copy(), dtype=np.float32)

    xyz = np.column_stack((x, y, z))
    # Filter out NaNs / Infs
    valid_mask = np.isfinite(xyz).all(axis=1)
    return xyz[valid_mask]


class SlopeTraversabilityCostmapNode(Node):
    def __init__(self):
        super().__init__('slope_traversability_costmap_node')

        # Parameters
        self.declare_parameter('grid_size_m', 8.0)         # 8m x 8m local grid around robot
        self.declare_parameter('grid_resolution_m', 0.10)   # 10cm cells
        self.declare_parameter('caution_slope_deg', 15.0)   # >= 15 deg = caution
        self.declare_parameter('lethal_slope_deg', 25.0)    # >= 25 deg = lethal obstacle
        self.declare_parameter('max_step_height_m', 0.14)   # >= 14cm step = lethal
        self.declare_parameter('min_points_per_cell', 3)
        self.declare_parameter('update_interval_sec', 0.2)  # 5 Hz

        self.grid_size = self.get_parameter('grid_size_m').value
        self.resolution = self.get_parameter('grid_resolution_m').value
        self.caution_slope = self.get_parameter('caution_slope_deg').value
        self.lethal_slope = self.get_parameter('lethal_slope_deg').value
        self.max_step = self.get_parameter('max_step_height_m').value
        self.min_pts = self.get_parameter('min_points_per_cell').value

        self.num_cells_1d = int(self.grid_size / self.resolution)
        self.half_size = self.grid_size / 2.0

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, 'slope_costmap/grid', 10)
        self.hazard_markers_pub = self.create_publisher(MarkerArray, 'slope_costmap/hazard_markers', 10)
        self.status_pub = self.create_publisher(String, 'slope_costmap/status', 10)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, 'platform/odom', self.odom_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'sensors/lidar3d_0/points', self.pointcloud_callback, sensor_qos
        )

        self.latest_cloud = None
        self.timer = self.create_timer(
            self.get_parameter('update_interval_sec').value, self.process_traversability
        )

        self.get_logger().info('Slope & Terrain Traversability Costmap Node initialized successfully.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def pointcloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg

    def process_traversability(self):
        if self.latest_cloud is None:
            return

        cloud_msg = self.latest_cloud
        xyz = pointcloud2_to_xyz_array(cloud_msg)
        if xyz.shape[0] < 50:
            return

        # Transform points into robot base local coordinates if in sensor frame
        # Sensor height is typically ~0.35m above ground
        xs = xyz[:, 0]
        ys = xyz[:, 1]
        zs = xyz[:, 2]

        # Crop to grid bounds
        in_bounds = (xs >= -self.half_size) & (xs <= self.half_size) & \
                    (ys >= -self.half_size) & (ys <= self.half_size) & \
                    (zs >= -0.8) & (zs <= 1.5)

        px = xs[in_bounds]
        py = ys[in_bounds]
        pz = zs[in_bounds]

        if len(px) == 0:
            return

        # Cell indices in local grid (0 to num_cells_1d - 1)
        grid_ix = np.clip(((px + self.half_size) / self.resolution).astype(np.int32), 0, self.num_cells_1d - 1)
        grid_iy = np.clip(((py + self.half_size) / self.resolution).astype(np.int32), 0, self.num_cells_1d - 1)

        # 2.5D Elevation arrays: z_min, z_max, z_sum, count
        grid_z_min = np.full((self.num_cells_1d, self.num_cells_1d), np.inf, dtype=np.float32)
        grid_z_max = np.full((self.num_cells_1d, self.num_cells_1d), -np.inf, dtype=np.float32)
        grid_z_sum = np.zeros((self.num_cells_1d, self.num_cells_1d), dtype=np.float32)
        grid_count = np.zeros((self.num_cells_1d, self.num_cells_1d), dtype=np.int32)

        # Accumulate points into 2.5D elevation cells
        for i in range(len(px)):
            gx = grid_ix[i]
            gy = grid_iy[i]
            z_val = pz[i]
            if z_val < grid_z_min[gx, gy]:
                grid_z_min[gx, gy] = z_val
            if z_val > grid_z_max[gx, gy]:
                grid_z_max[gx, gy] = z_val
            grid_z_sum[gx, gy] += z_val
            grid_count[gx, gy] += 1

        # Mean elevation surface
        valid_mask = grid_count >= self.min_pts
        elevation_map = np.zeros((self.num_cells_1d, self.num_cells_1d), dtype=np.float32)
        elevation_map[valid_mask] = grid_z_sum[valid_mask] / grid_count[valid_mask]

        # Step discontinuity: delta_z = z_max - z_min
        step_height_map = np.zeros((self.num_cells_1d, self.num_cells_1d), dtype=np.float32)
        step_height_map[valid_mask] = grid_z_max[valid_mask] - grid_z_min[valid_mask]

        # Calculate slope angles via finite differences
        # dz/dx and dz/dy
        grad_x = np.zeros_like(elevation_map)
        grad_y = np.zeros_like(elevation_map)

        grad_x[1:-1, :] = (elevation_map[2:, :] - elevation_map[:-2, :]) / (2.0 * self.resolution)
        grad_y[:, 1:-1] = (elevation_map[:, 2:] - elevation_map[:, :-2]) / (2.0 * self.resolution)

        slope_rad = np.arctan(np.sqrt(grad_x**2 + grad_y**2))
        slope_deg = np.degrees(slope_rad)

        # Build Costmap:
        # -1 = Unknown
        # 0 = Free / Flat
        # 40..80 = Caution Slope
        # 100 = Lethal Obstacle (Steep slope or step drop-off)
        cost_grid = np.full((self.num_cells_1d, self.num_cells_1d), -1, dtype=np.int8)

        # Free space
        cost_grid[valid_mask & (slope_deg < self.caution_slope) & (step_height_map < self.max_step)] = 0

        # Caution Slope
        caution_mask = valid_mask & (slope_deg >= self.caution_slope) & (slope_deg < self.lethal_slope)
        cost_grid[caution_mask] = 60

        # Lethal Slope or Obstacle Step
        lethal_mask = valid_mask & ((slope_deg >= self.lethal_slope) | (step_height_map >= self.max_step))
        cost_grid[lethal_mask] = 100

        # Count hazards
        lethal_count = int(np.sum(lethal_mask))
        caution_count = int(np.sum(caution_mask))
        max_observed_slope = float(np.max(slope_deg[valid_mask])) if np.any(valid_mask) else 0.0

        # Publish OccupancyGrid
        costmap_msg = OccupancyGrid()
        costmap_msg.header = Header()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = 'base_link'

        costmap_msg.info.resolution = float(self.resolution)
        costmap_msg.info.width = int(self.num_cells_1d)
        costmap_msg.info.height = int(self.num_cells_1d)
        costmap_msg.info.origin.position.x = -float(self.half_size)
        costmap_msg.info.origin.position.y = -float(self.half_size)
        costmap_msg.info.origin.position.z = -0.3
        costmap_msg.info.origin.orientation.w = 1.0

        # Flatten row-major
        costmap_msg.data = cost_grid.T.flatten().tolist()
        self.costmap_pub.publish(costmap_msg)

        # Publish 3D Hazard Markers for Lethal Slopes
        self.publish_hazard_markers(lethal_mask, elevation_map)

        # Publish status telemetry
        status_msg = String()
        status_msg.data = f"SLOPE_MAP: Max={max_observed_slope:.1f}deg | LethalCells={lethal_count} | CautionCells={caution_count}"
        self.status_pub.publish(status_msg)

    def publish_hazard_markers(self, lethal_mask: np.ndarray, elevation_map: np.ndarray):
        marker_array = MarkerArray()

        # Delete old markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        # Subsample lethal cells for markers
        lx, ly = np.where(lethal_mask)
        max_markers = 40
        step = max(1, len(lx) // max_markers)

        for idx in range(0, len(lx), step):
            gx = lx[idx]
            gy = ly[idx]

            mx = (gx * self.resolution) - self.half_size + (self.resolution / 2.0)
            my = (gy * self.resolution) - self.half_size + (self.resolution / 2.0)
            mz = float(elevation_map[gx, gy]) + 0.2

            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'slope_hazards'
            marker.id = idx + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = float(mx)
            marker.pose.position.y = float(my)
            marker.pose.position.z = float(mz)
            marker.pose.orientation.w = 1.0

            marker.scale.x = float(self.resolution * 0.9)
            marker.scale.y = float(self.resolution * 0.9)
            marker.scale.z = 0.35

            # Red hazard
            marker.color.r = 0.95
            marker.color.g = 0.15
            marker.color.b = 0.15
            marker.color.a = 0.85

            marker_array.markers.append(marker)

        self.hazard_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SlopeTraversabilityCostmapNode()
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
