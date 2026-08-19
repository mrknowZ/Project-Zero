#!/usr/bin/env python3
"""
Project Zero — Complete Live System & Mapping Diagnostics
Tests LiDAR rates, SLAM Map generation, TF frame connectivity, and robot transforms.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros


class SystemDiagnostic(Node):
    def __init__(self, namespace="j100_0751"):
        super().__init__('live_system_diagnostic')
        self.namespace = namespace
        self.scan_count = 0
        self.map_count = 0
        self.odom_count = 0
        self.latest_map_info = None

        scan_topic = f"/{namespace}/sensors/lidar3d_0/scan"
        map_topic = f"/{namespace}/map"
        odom_topic = f"/{namespace}/platform/odom"

        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.create_subscription(OccupancyGrid, map_topic, self.map_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def scan_cb(self, msg):
        self.scan_count += 1

    def map_cb(self, msg):
        self.map_count += 1
        self.latest_map_info = f"{msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell"

    def odom_cb(self, msg):
        self.odom_count += 1


def main():
    rclpy.init()
    diag = SystemDiagnostic()

    print("\n" + "=" * 65)
    print("🔍 RUNNING LIVE ROBOT & SLAM DIAGNOSTICS (Listening for 4s)...")
    print("=" * 65)

    start = time.time()
    while time.time() - start < 4.0:
        rclpy.spin_once(diag, timeout_sec=0.1)

    print("\n📊 SENSOR & TOPIC HEALTH:")
    scan_status = "✅ ACTIVE" if diag.scan_count > 0 else "❌ NO DATA"
    map_status = "✅ ACTIVE" if diag.map_count > 0 else "❌ NO DATA"
    odom_status = "✅ ACTIVE" if diag.odom_count > 0 else "❌ NO DATA"

    print(f" • 3D LiDAR Scan (/j100_0751/sensors/lidar3d_0/scan):  {scan_status} ({diag.scan_count} msgs received)")
    print(f" • SLAM Map       (/j100_0751/map):                     {map_status} ({diag.map_count} msgs received)")
    if diag.latest_map_info:
        print(f"   ↳ Map Dimensions: {diag.latest_map_info}")
    print(f" • Wheel Odom     (/j100_0751/platform/odom):           {odom_status} ({diag.odom_count} msgs received)")

    print("\n🌲 TF TRANSFORM TREE CONNECTIVITY:")
    # Check odom -> base_link
    odom_to_base = False
    try:
        diag.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        odom_to_base = True
        print(" • [odom -> base_link]:     ✅ CONNECTED")
    except Exception as e:
        print(f" • [odom -> base_link]:     ❌ DISCONNECTED ({e})")

    # Check map -> odom
    map_to_odom = False
    try:
        diag.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        map_to_odom = True
        print(" • [map -> odom]:          ✅ CONNECTED")
    except Exception as e:
        print(f" • [map -> odom]:          ❌ DISCONNECTED ({e})")

    # Check map -> base_link
    try:
        diag.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        print(" • [map -> base_link]:     ✅ CONNECTED (Full localization active)")
    except Exception as e:
        print(f" • [map -> base_link]:     ❌ DISCONNECTED ({e})")

    print("\n" + "=" * 65)
    if map_status.startswith("✅") and odom_to_base and map_to_odom:
        print("🎉 ALL SYSTEMS GREEN! SLAM and Navigation are fully working.")
    else:
        print("⚠️ ACTION ITEMS:")
        if diag.scan_count == 0:
            print(" - Restart sensors service: sudo systemctl restart clearpath-sensors.service")
        if not odom_to_base:
            print(" - Restart platform service: sudo systemctl restart clearpath-platform.service")
        if diag.map_count == 0:
            print(" - Check if SLAM Toolbox is running with correct scan_topic")
    print("=" * 65 + "\n")

    diag.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
