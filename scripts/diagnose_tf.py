#!/usr/bin/env python3
"""
Project Zero — SAFiR Lab
TF Tree & Hardware Diagnostic Tool

Inspects active TF frames, rates, namespaces, and connectivity across all topics.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import time
import sys


class TFDiagnostic(Node):
    def __init__(self):
        super().__init__('tf_diagnostic')
        self.frames = {}
        self.sources = {}

        # Subscribe to all possible TF topics
        self.create_subscription(TFMessage, '/tf', lambda msg: self.tf_cb(msg, '/tf'), 50)
        self.create_subscription(TFMessage, '/tf_static', lambda msg: self.tf_cb(msg, '/tf_static'), 50)
        self.create_subscription(TFMessage, '/j100_0751/tf', lambda msg: self.tf_cb(msg, '/j100_0751/tf'), 50)
        self.create_subscription(TFMessage, '/j100_0751/tf_static', lambda msg: self.tf_cb(msg, '/j100_0751/tf_static'), 50)

    def tf_cb(self, msg: TFMessage, topic: str):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            key = f"{parent} -> {child}"
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            now = time.time()
            dt = now - stamp
            self.frames[key] = (topic, dt, stamp)


def main():
    rclpy.init()
    node = TFDiagnostic()
    print("=" * 60)
    print("Listening to TF broadcasts for 4 seconds...")
    print("=" * 60)
    
    start = time.time()
    while rclpy.ok() and (time.time() - start < 4.0):
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n" + "=" * 60)
    print(f"DIAGNOSTIC REPORT: Found {len(node.frames)} TF Transform Pairs:")
    print("=" * 60)
    
    if not node.frames:
        print("❌ NO TF TRANSFORMS DETECTED ON ANY TOPIC!")
        print("Check if robot base services (clearpath-platform.service) are running.")
    else:
        for pair, (topic, dt, stamp) in sorted(node.frames.items()):
            status = "✅ OK" if abs(dt) < 2.0 else f"⚠️ TIME DRIFT ({dt:+.2f}s)"
            print(f" • {pair:35s} | Topic: {topic:20s} | {status}")

    print("=" * 60)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
