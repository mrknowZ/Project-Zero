#!/usr/bin/env python3
"""
Project Zero — Laptop-to-Robot Network & Topic Diagnostic Tool
Tests if your laptop can reach the robot over the network and discover robot topics.
"""

import os
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node


def check_ping(ip):
    try:
        res = subprocess.run(['ping', '-c', '2', '-W', '2', ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return res.returncode == 0
    except Exception:
        return False


class TopicChecker(Node):
    def __init__(self, namespace="j100_0751"):
        super().__init__('laptop_diagnostic')
        self.namespace = namespace
        self.received_topics = set()
        
        # Poll topic names and types
        self.timer = self.create_timer(0.5, self.check_topics)

    def check_topics(self):
        topic_list = self.get_topic_names_and_types()
        for name, _ in topic_list:
            if self.namespace in name:
                self.received_topics.add(name)


def main():
    print("\n" + "=" * 65)
    print("🔍 RUNNING LAPTOP-TO-ROBOT DIAGNOSTIC...")
    print("=" * 65)

    # 1. Check Domain ID
    domain_id = os.environ.get('ROS_DOMAIN_ID', 'Not set (defaults to 0)')
    print(f"\n📡 Active ROS_DOMAIN_ID on Laptop: {domain_id}")
    if domain_id != '0':
        print("⚠️ WARNING: ROS_DOMAIN_ID must be 0 to talk to Clearpath hardware!")

    # 2. Check Physical Network Connectivity (Ethernet / Wi-Fi)
    test_ips = ["192.168.131.1", "10.160.224.41"]
    reachable_ip = None
    print("\n🌐 TESTING NETWORK PING TO ROBOT:")
    for ip in test_ips:
        if check_ping(ip):
            print(f" • Robot IP ({ip}): ✅ REACHABLE")
            reachable_ip = ip
        else:
            print(f" • Robot IP ({ip}): ❌ UNREACHABLE")

    # 3. Check ROS 2 Discovery
    print("\n🎧 LISTENING FOR ROBOT ROS 2 TOPICS (Listening for 4s)...")
    rclpy.init()
    checker = TopicChecker()
    start = time.time()
    while time.time() - start < 4.0:
        rclpy.spin_once(checker, timeout_sec=0.2)

    topics = sorted(checker.received_topics)
    print(f"\n📊 DISCOVERED ROBOT TOPICS ({len(topics)} found):")
    if not topics:
        print(" ❌ NO ROBOT TOPICS RECEIVED!")
        print(" ↳ Reason: WSL2 virtual NAT is blocking ROS 2 multicast discovery.")
        print(f"\n💡 ONE-LINE FIX: Run this to connect directly to robot ({reachable_ip or '192.168.131.1'}):")
        print(" -------------------------------------------------------------")
        print(f" sudo apt install -y ros-humble-rmw-cyclonedds-cpp")
        print(f" export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
        print(f" export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Discovery><Peers><Peer address=\"{reachable_ip or '192.168.131.1'}\"/></Peers></Discovery></General></Domain></CycloneDDS>'")
        print(" -------------------------------------------------------------")
    else:
        for t in topics:
            print(f"  • {t}")
        print("\n🎉 SUCCESS! All robot topics are reaching your laptop.")

    print("=" * 65 + "\n")
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
