#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

def main():
    rclpy.init()
    node = Node('auto_initial_pose_publisher')
    
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    
    pub = node.create_publisher(PoseWithCovarianceStamped, '/j100_0000/initialpose', qos)
    
    # Wait for AMCL subscriber to connect
    node.get_logger().info('Waiting for AMCL initialpose subscriber...')
    timeout = time.time() + 30.0
    while time.time() < timeout:
        if pub.get_subscription_count() > 0:
            break
        time.sleep(0.5)
        
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    
    for _ in range(5):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.2)
        
    node.get_logger().info('Published initial pose (0,0,0) to /j100_0000/initialpose')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
