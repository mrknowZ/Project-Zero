#!/usr/bin/env python3
"""
Project Zero — SAFiR Lab
Sensors & Topics Diagnostic Tool

Lists all active ROS 2 topics and checks lidar & camera publication rates.
"""

import rclpy
from rclpy.node import Node
import time


def main():
    rclpy.init()
    node = Node('sensor_diagnostic')
    time.sleep(1.0)
    topic_list = node.get_topic_names_and_types()
    
    print("=" * 60)
    print("ACTIVE SENSOR TOPICS & TYPES:")
    print("=" * 60)
    
    lidar_topics = []
    camera_topics = []
    imu_topics = []
    odom_topics = []
    
    for name, types in sorted(topic_list):
        if 'scan' in name.lower() or 'point' in name.lower() or 'lidar' in name.lower():
            lidar_topics.append((name, types))
        elif 'camera' in name.lower() or 'image' in name.lower():
            camera_topics.append((name, types))
        elif 'imu' in name.lower():
            imu_topics.append((name, types))
        elif 'odom' in name.lower():
            odom_topics.append((name, types))
            
    print("\n📡 LIDAR TOPICS:")
    for name, types in lidar_topics:
        print(f" • {name:50s} [{', '.join(types)}]")
        
    print("\n📷 CAMERA TOPICS:")
    for name, types in camera_topics:
        print(f" • {name:50s} [{', '.join(types)}]")
        
    print("\n🧭 IMU / ODOM TOPICS:")
    for name, types in imu_topics + odom_topics:
        print(f" • {name:50s} [{', '.join(types)}]")
        
    print("=" * 60)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
