"""
Project Zero — SAFiR Lab
Mission Orchestrator Node

Executes an end-to-end autonomous mission:
  1. Waits for Nav2 to come up and for AMCL to localize.
  2. Navigates through a list of waypoints sequentially.
  3. At each waypoint, pauses briefly to collect object detections from
     the jackal_vision node.
  4. After visiting all waypoints (or on shutdown), writes a summary
     report (JSON + human-readable log).

Uses the nav2_simple_commander BasicNavigator API.
"""

import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class MissionNode(Node):
    """Orchestrates a multi-waypoint navigation + detection mission."""

    def __init__(self):
        super().__init__('mission_node')

        # --------------- Parameters ---------------
        self.declare_parameter('namespace', 'j100_0000')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('dwell_time_sec', 5.0)
        self.declare_parameter('report_dir', '/tmp/jackal_mission_reports')
        self.declare_parameter(
            'detection_topic',
            '/j100_0000/yolo_detector/detections',
        )

        self.namespace = self.get_parameter('namespace').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        waypoints_file = self.get_parameter('waypoints_file').value
        self.dwell_time = self.get_parameter('dwell_time_sec').value
        self.report_dir = self.get_parameter('report_dir').value
        det_topic = self.get_parameter('detection_topic').value

        # --------------- Load waypoints ---------------
        if waypoints_file and os.path.isfile(waypoints_file):
            self.waypoints = self._load_waypoints(waypoints_file)
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from {waypoints_file}'
            )
        else:
            # Default demo waypoints for the warehouse world
            self.waypoints = [
                {'name': 'wp1', 'x':  2.0, 'y':  1.0, 'yaw': 0.0},
                {'name': 'wp2', 'x':  5.0, 'y': -2.0, 'yaw': 1.57},
                {'name': 'wp3', 'x': -1.0, 'y': -3.0, 'yaw': 3.14},
            ]
            self.get_logger().warn(
                'No waypoints file provided — using default demo waypoints. '
                'Override with waypoints_file parameter.'
            )

        # --------------- Detection subscriber ---------------
        self.latest_detections: list[dict] = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._detection_callback, qos
        )

        # --------------- Mission log ---------------
        self.mission_log: list[dict] = []

        self.get_logger().info('MissionNode initialised — call run_mission()')

    # ------------------------------------------------------------------
    def _load_waypoints(self, filepath: str) -> list[dict]:
        """Load waypoints from a YAML file."""
        import yaml
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', [])

    # ------------------------------------------------------------------
    def _detection_callback(self, msg: Detection2DArray):
        """Cache the most recent detections."""
        self.latest_detections = []
        for det in msg.detections:
            if det.results:
                hyp = det.results[0].hypothesis
                self.latest_detections.append({
                    'class': hyp.class_id,
                    'score': round(float(hyp.score), 3),
                    'bbox': {
                        'cx': round(float(det.bbox.center.position.x), 1),
                        'cy': round(float(det.bbox.center.position.y), 1),
                        'w':  round(float(det.bbox.size_x), 1),
                        'h':  round(float(det.bbox.size_y), 1),
                    },
                })

    # ------------------------------------------------------------------
    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Create a PoseStamped from x, y, yaw (radians)."""
        import math
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        if self.use_sim_time:
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0
        else:
            pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        # Convert yaw to quaternion (only z-rotation)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ------------------------------------------------------------------
    def run_mission(self):
        """Execute the full waypoint-following + detection mission."""
        navigator = BasicNavigator(namespace=f'/{self.namespace}')
        if self.use_sim_time:
            navigator.set_parameters([
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ])

        # Initialize AMCL pose
        init_pose = self._make_pose(0.0, 0.0, 0.0)
        navigator.setInitialPose(init_pose)

        # Wait for Nav2 to activate
        self.get_logger().info('Waiting for Nav2 to become active…')
        navigator.waitUntilNav2Active(localizer='amcl')
        self.get_logger().info('Nav2 is active — starting mission.')

        for i, wp in enumerate(self.waypoints):
            wp_name = wp.get('name', f'waypoint_{i}')
            x, y, yaw = wp['x'], wp['y'], wp.get('yaw', 0.0)

            self.get_logger().info(
                f'[{i+1}/{len(self.waypoints)}] Navigating to {wp_name} '
                f'({x}, {y}, yaw={yaw:.2f})'
            )

            goal_pose = self._make_pose(x, y, yaw)
            navigator.goToPose(goal_pose)

            # Poll until navigation completes
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    eta = Duration.from_msg(
                        feedback.estimated_time_remaining
                    ).nanoseconds / 1e9
                    self.get_logger().info(
                        f'  ETA to {wp_name}: {eta:.1f}s', throttle_duration_sec=2.0
                    )
                time.sleep(0.25)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'✓ Reached {wp_name}')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'✗ Navigation to {wp_name} was cancelled')
            elif result == TaskResult.FAILED:
                self.get_logger().warn(
                    f'✗ Navigation to {wp_name} failed — skipping'
                )

            # Dwell at waypoint to collect detections
            self.get_logger().info(
                f'  Dwelling at {wp_name} for {self.dwell_time}s to collect detections…'
            )
            self.latest_detections = []
            dwell_start = time.time()
            all_detections_at_wp: list[dict] = []
            while time.time() - dwell_start < self.dwell_time:
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.latest_detections:
                    all_detections_at_wp.extend(self.latest_detections)
                    self.latest_detections = []

            # De-duplicate by class name (keep highest confidence)
            unique = {}
            for d in all_detections_at_wp:
                cls = d['class']
                if cls not in unique or d['score'] > unique[cls]['score']:
                    unique[cls] = d
            deduped = list(unique.values())

            self.mission_log.append({
                'waypoint': wp_name,
                'position': {'x': x, 'y': y, 'yaw': yaw},
                'nav_result': str(result),
                'detections': deduped,
            })

            if deduped:
                classes = [d['class'] for d in deduped]
                self.get_logger().info(
                    f'  Detected at {wp_name}: {classes}'
                )
            else:
                self.get_logger().info(f'  No detections at {wp_name}.')

        # ---- Generate report ----
        self._write_report()
        self.get_logger().info('Mission complete!')

    # ------------------------------------------------------------------
    def _write_report(self):
        """Write the mission report to disk."""
        os.makedirs(self.report_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # JSON report
        json_path = os.path.join(self.report_dir, f'mission_{timestamp}.json')
        with open(json_path, 'w') as f:
            json.dump(self.mission_log, f, indent=2)
        self.get_logger().info(f'JSON report saved: {json_path}')

        # Human-readable log
        log_path = os.path.join(self.report_dir, f'mission_{timestamp}.txt')
        with open(log_path, 'w') as f:
            f.write(f'SAFiR Lab — Project Zero Mission Report\n')
            f.write(f'Generated: {datetime.now().isoformat()}\n')
            f.write(f'{"=" * 50}\n\n')
            total_dets = 0
            for entry in self.mission_log:
                wp = entry['waypoint']
                pos = entry['position']
                f.write(f'Waypoint: {wp}  ({pos["x"]}, {pos["y"]})\n')
                f.write(f'  Navigation: {entry["nav_result"]}\n')
                dets = entry['detections']
                if dets:
                    for d in dets:
                        f.write(
                            f'  Detected: {d["class"]} '
                            f'(confidence={d["score"]})\n'
                        )
                        total_dets += 1
                else:
                    f.write('  No objects detected.\n')
                f.write('\n')
            f.write(f'{"=" * 50}\n')
            f.write(
                f'Total waypoints: {len(self.mission_log)}  |  '
                f'Total detections: {total_dets}\n'
            )
        self.get_logger().info(f'Text report saved: {log_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
