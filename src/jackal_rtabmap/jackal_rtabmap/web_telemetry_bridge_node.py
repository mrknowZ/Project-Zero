#!/usr/bin/env python3
"""
Web & Tablet Telemetry Bridge Node for Clearpath Jackal J100
Serves a professional, human-designed industrial UI on port 8080.
Bridges ROS 2 topics:
  - Odometry & Velocity
  - 6-DoF Terrain Slope & Inclinometer
  - 3D Semantic Object Landmark Registry
  - High-Reliability Camera Feed (Dual-Stream YOLO / Raw Color Fallback)
  - Remote Teleoperation & Goal Navigation Dispatch
  - Map Saving Service
"""

import json
import math
import os
import subprocess
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tornado.web
import tornado.websocket
import tornado.ioloop

from geometry_msgs.msg import Twist, Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


def imgmsg_to_cv2(img_msg: Image) -> np.ndarray:
    """Robust conversion of ROS Image message to OpenCV numpy array."""
    if img_msg.encoding == 'bgr8':
        return np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3)).copy()
    elif img_msg.encoding == 'rgb8':
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
        return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    elif '32F' in img_msg.encoding or '32FC1' in img_msg.encoding:
        return np.frombuffer(img_msg.data, dtype=np.float32).reshape((img_msg.height, img_msg.width)).copy()
    else:
        im = np.frombuffer(img_msg.data, dtype=np.uint8)
        if im.size == img_msg.height * img_msg.width * 3:
            return im.reshape((img_msg.height, img_msg.width, 3)).copy()
        return None


# Global Telemetry State
state = {
    'slope': {'roll': 0.0, 'pitch': 0.0, 'total': 0.0, 'status': 'SAFE'},
    'odom': {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'speed': 0.0, 'angular': 0.0},
    'exploration': {'status': 'AUTONOMOUS NAVIGATION READY', 'frontiers': 0, 'target': None},
    'landmarks': [],
    'latest_frame_jpeg': None,
    'has_yolo': False,
    'map_save_status': 'IDLE',
}

ws_clients = set()
ros_node = None


class IndexHandler(tornado.web.RequestHandler):
    def get(self):
        try:
            with open('/home/holetown/ali/Project-Zero/src/jackal_rtabmap/www/index.html', 'rb') as f:
                data = f.read()
            self.set_header('Content-Type', 'text/html; charset=utf-8')
            self.write(data)
        except Exception as e:
            self.set_header('Content-Type', 'text/plain')
            self.write(f'Index Error: {e}')

    def head(self):
        self.set_header('Content-Type', 'text/html; charset=utf-8')
        self.set_status(200)


class VideoFeedHandler(tornado.web.RequestHandler):
    async def get(self):
        self.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.set_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.set_header('Pragma', 'no-cache')
        self.set_header('Expires', '0')

        while True:
            if state['latest_frame_jpeg'] is not None:
                frame = state['latest_frame_jpeg']
                self.write(b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                await self.flush()
            await tornado.gen.sleep(0.05)  # 20 FPS


class TelemetryWebSocket(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        ws_clients.add(self)

    def on_close(self):
        ws_clients.remove(self)

    def on_message(self, message):
        try:
            data = json.loads(message)
            msg_type = data.get('type')

            if msg_type == 'cmd_vel' and ros_node is not None:
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
                ros_node.publish_cmd_vel(linear, angular)

            elif msg_type == 'nav_goal' and ros_node is not None:
                gx = float(data.get('x', 0.0))
                gy = float(data.get('y', 0.0))
                ros_node.send_navigation_goal(gx, gy)

            elif msg_type == 'save_map':
                threading.Thread(target=save_map_task, daemon=True).start()

        except Exception:
            pass


def save_map_task():
    try:
        maps_dir = '/home/holetown/ali/Project-Zero/maps'
        os.makedirs(maps_dir, exist_ok=True)
        map_path = os.path.join(maps_dir, 'jackal_farm_map')
        cmd = f"source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f {map_path} --ros-args -r __ns:=/j100_0000"
        subprocess.run(cmd, shell=True, executable='/bin/bash', timeout=15)
        state['map_save_status'] = 'SAVED_OK'
    except Exception as e:
        state['map_save_status'] = f'ERROR: {e}'


def broadcast_telemetry():
    if not ws_clients:
        return

    payload = json.dumps({
        'slope': state['slope'],
        'odom': state['odom'],
        'exploration': state['exploration'],
        'landmarks': state['landmarks'],
        'has_yolo': state['has_yolo'],
        'map_save_status': state['map_save_status'],
    })

    for client in list(ws_clients):
        try:
            client.write_message(payload)
        except Exception:
            pass


class WebTelemetryBridgeNode(Node):
    def __init__(self):
        super().__init__('web_telemetry_bridge_node')
        global ros_node
        ros_node = self

        # Teleop & Navigation publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.slope_sub = self.create_subscription(
            Vector3Stamped, 'slope_safety/telemetry', self.slope_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, 'slope_safety/status', self.slope_status_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'platform/odom', self.odom_callback, 10
        )
        self.exploration_sub = self.create_subscription(
            String, 'exploration/status', self.exploration_callback, 10
        )
        self.landmarks_sub = self.create_subscription(
            String, 'semantic_map/objects_json', self.landmarks_callback, 10
        )

        # Dual Image Subscriptions: YOLO detections + Raw Camera Fallback
        self.yolo_image_sub = self.create_subscription(
            Image, 'yolo_detector/detections_image', self.yolo_image_callback, sensor_qos
        )
        self.raw_image_sub = self.create_subscription(
            Image, 'sensors/camera_0/image_color', self.raw_image_callback, sensor_qos
        )

        self.get_logger().info('Web Industrial Telemetry Bridge Node initialized.')

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = max(-0.8, min(0.8, linear))
        twist.angular.z = max(-1.2, min(1.2, angular))
        self.cmd_vel_pub.publish(twist)

    def send_navigation_goal(self, gx, gy):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Dispatched Nav2 Goal via Web: ({gx:.2f}, {gy:.2f})')

    def slope_callback(self, msg: Vector3Stamped):
        state['slope']['roll'] = round(msg.vector.x, 1)
        state['slope']['pitch'] = round(msg.vector.y, 1)
        state['slope']['total'] = round(msg.vector.z, 1)

    def slope_status_callback(self, msg: String):
        raw = msg.data
        if 'DANGER' in raw:
            state['slope']['status'] = 'DANGER'
        elif 'CAUTION' in raw:
            state['slope']['status'] = 'CAUTION'
        else:
            state['slope']['status'] = 'SAFE'

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        state['odom']['x'] = round(pos.x, 2)
        state['odom']['y'] = round(pos.y, 2)

        # Yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        state['odom']['yaw'] = round(yaw, 1)
        state['odom']['speed'] = round(msg.twist.twist.linear.x, 2)
        state['odom']['angular'] = round(msg.twist.twist.angular.z, 2)

    def exploration_callback(self, msg: String):
        state['exploration']['status'] = msg.data

    def landmarks_callback(self, msg: String):
        try:
            state['landmarks'] = json.loads(msg.data)
        except Exception:
            pass

    def yolo_image_callback(self, msg: Image):
        try:
            cv_img = imgmsg_to_cv2(msg)
            if cv_img is not None:
                state['has_yolo'] = True
                small_img = cv2.resize(cv_img, (640, 360))
                _, buffer = cv2.imencode('.jpg', small_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                state['latest_frame_jpeg'] = buffer.tobytes()
        except Exception:
            pass

    def raw_image_callback(self, msg: Image):
        # Fallback to raw camera feed if YOLO detections not yet published
        if not state['has_yolo'] or state['latest_frame_jpeg'] is None:
            try:
                cv_img = imgmsg_to_cv2(msg)
                if cv_img is not None:
                    small_img = cv2.resize(cv_img, (640, 360))
                    _, buffer = cv2.imencode('.jpg', small_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    state['latest_frame_jpeg'] = buffer.tobytes()
            except Exception:
                pass


def start_tornado_server():
    import asyncio
    asyncio.set_event_loop(asyncio.new_event_loop())
    www_dir = '/home/holetown/ali/Project-Zero/src/jackal_rtabmap/www'
    app = tornado.web.Application([
        (r'/', IndexHandler),
        (r'/video_feed', VideoFeedHandler),
        (r'/ws', TelemetryWebSocket),
        (r'/(.*)', tornado.web.StaticFileHandler, {'path': www_dir, 'default_filename': 'index.html'}),
    ])

    port = 8080
    app.listen(port)
    print(f'>>> Jackal Web Industrial Mission Control online at http://localhost:{port}')

    tornado.ioloop.PeriodicCallback(broadcast_telemetry, 100).start()
    tornado.ioloop.IOLoop.current().start()


def main(args=None):
    rclpy.init(args=args)
    node = WebTelemetryBridgeNode()

    # Start Tornado event loop in separate background thread
    server_thread = threading.Thread(target=start_tornado_server, daemon=True)
    server_thread.start()

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
