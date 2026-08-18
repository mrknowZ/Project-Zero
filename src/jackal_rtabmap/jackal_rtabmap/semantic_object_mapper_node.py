#!/usr/bin/env python3
"""
3D Semantic Object Mapper Node for Clearpath Jackal J100
Projects YOLOv8 2D camera detections into 3D world space (map frame)
using RGB-D depth correlation and TF transforms.
Publishes 3D bounding boxes and persistent semantic landmarks.
"""

import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, ColorRGBA
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


def imgmsg_to_cv2(img_msg: Image) -> np.ndarray:
    """Pure-Python conversion of ROS Image message to OpenCV numpy array without cv_bridge."""
    if img_msg.encoding == 'bgr8':
        return np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3)).copy()
    elif img_msg.encoding == 'rgb8':
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
        return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    elif '32F' in img_msg.encoding or '32FC1' in img_msg.encoding:
        return np.frombuffer(img_msg.data, dtype=np.float32).reshape((img_msg.height, img_msg.width)).copy()
    elif '16U' in img_msg.encoding or '16UC1' in img_msg.encoding:
        d16 = np.frombuffer(img_msg.data, dtype=np.uint16).reshape((img_msg.height, img_msg.width))
        return d16.astype(np.float32) / 1000.0
    else:
        im = np.frombuffer(img_msg.data, dtype=np.uint8)
        if im.size == img_msg.height * img_msg.width * 3:
            return im.reshape((img_msg.height, img_msg.width, 3)).copy()
        elif im.size == img_msg.height * img_msg.width:
            return im.reshape((img_msg.height, img_msg.width)).copy()
        return None


class SemanticObjectMapperNode(Node):
    def __init__(self):
        super().__init__('semantic_object_mapper_node')

        # Parameters
        self.declare_parameter('camera_frame', 'camera_0_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('clustering_distance', 0.6)  # meters

        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.clustering_distance = self.get_parameter('clustering_distance').value

        self.latest_depth_image = None
        self.camera_intrinsics = None  # (fx, fy, cx, cy)

        # Persistent Semantic Landmarks Registry: List of dicts
        # [{'id': int, 'label': str, 'score': float, 'x': float, 'y': float, 'z': float, 'count': int}]
        self.landmarks = []
        self.landmark_id_counter = 1

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'semantic_map/markers', 10)
        self.json_pub = self.create_publisher(String, 'semantic_map/objects_json', 10)

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, 'sensors/camera_0/depth_image', self.depth_callback, sensor_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'sensors/camera_0/camera_info', self.camera_info_callback, sensor_qos
        )
        self.detections_sub = self.create_subscription(
            Detection2DArray, 'yolo_detector/detections', self.detections_callback, 10
        )

        # Periodic Landmark Publisher (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_landmarks)

        self.get_logger().info('3D Semantic Object Mapper Node initialized.')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_intrinsics is None:
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            if fx > 0 and fy > 0:
                self.camera_intrinsics = (fx, fy, cx, cy)
                self.get_logger().info(f'Camera intrinsics calibrated: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_image = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().debug(f'Depth conversion error: {e}')

    def detections_callback(self, msg: Detection2DArray):
        if self.latest_depth_image is None or self.camera_intrinsics is None:
            return

        fx, fy, cx, cy = self.camera_intrinsics
        depth_img = self.latest_depth_image
        h, w = depth_img.shape[:2]

        for det in msg.detections:
            if not det.results:
                continue

            label = det.results[0].hypothesis.class_id
            score = float(det.results[0].hypothesis.score)

            # Center pixel of bounding box
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            if not (0 <= u < w and 0 <= v < h):
                continue

            # Sample depth in a 5x5 window around center to reject noise
            u_min = max(0, u - 2)
            u_max = min(w, u + 3)
            v_min = max(0, v - 2)
            v_max = min(h, v + 3)

            depth_patch = depth_img[v_min:v_max, u_min:u_max]
            valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0.3) & (depth_patch < 15.0)]

            if len(valid_depths) == 0:
                continue

            z_cam = float(np.median(valid_depths))
            x_cam = (u - cx) * z_cam / fx
            y_cam = (v - cy) * z_cam / fy

            # Transform from camera frame to map frame
            map_pt = self.transform_camera_to_map(x_cam, y_cam, z_cam)
            if map_pt is not None:
                self.integrate_landmark(label, score, map_pt[0], map_pt[1], map_pt[2])

    def transform_camera_to_map(self, x_cam, y_cam, z_cam):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            trans = t.transform.translation
            rot = t.transform.rotation

            # Quaternion to rotation matrix
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            r11 = 1 - 2 * (qy * qy + qz * qz)
            r12 = 2 * (qx * qy - qz * qw)
            r13 = 2 * (qx * qz + qy * qw)
            r21 = 2 * (qx * qy + qz * qw)
            r22 = 1 - 2 * (qx * qx + qz * qz)
            r23 = 2 * (qy * qz - qx * qw)
            r31 = 2 * (qx * qz - qy * qw)
            r32 = 2 * (qy * qz + qx * qw)
            r33 = 1 - 2 * (qx * qx + qy * qy)

            xm = r11 * x_cam + r12 * y_cam + r13 * z_cam + trans.x
            ym = r21 * x_cam + r22 * y_cam + r23 * z_cam + trans.y
            zm = r31 * x_cam + r32 * y_cam + r33 * z_cam + trans.z

            return (xm, ym, zm)
        except Exception:
            return None

    def integrate_landmark(self, label, score, x, y, z):
        matched = False
        for lm in self.landmarks:
            if lm['label'] == label:
                dist = math.hypot(lm['x'] - x, lm['y'] - y)
                if dist <= self.clustering_distance:
                    c = lm['count']
                    lm['x'] = (lm['x'] * c + x) / (c + 1)
                    lm['y'] = (lm['y'] * c + y) / (c + 1)
                    lm['z'] = (lm['z'] * c + z) / (c + 1)
                    lm['score'] = max(lm['score'], score)
                    lm['count'] += 1
                    matched = True
                    break

        if not matched:
            new_lm = {
                'id': self.landmark_id_counter,
                'label': label,
                'score': round(score, 2),
                'x': round(x, 2),
                'y': round(y, 2),
                'z': round(z, 2),
                'count': 1,
            }
            self.landmarks.append(new_lm)
            self.landmark_id_counter += 1
            self.get_logger().info(f'Discovered new 3D Landmark: {label} at ({x:.2f}, {y:.2f}, {z:.2f}) [Score: {score:.2f}]')

    def publish_landmarks(self):
        if not self.landmarks:
            return

        # 1. Publish JSON String
        json_msg = String()
        json_msg.data = json.dumps(self.landmarks)
        self.json_pub.publish(json_msg)

        # 2. Publish 3D Marker Array to RViz
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for lm in self.landmarks:
            lid = lm['id']
            # Marker A: 3D Bounding Box Cube
            box_marker = Marker()
            box_marker.header.stamp = now
            box_marker.header.frame_id = self.map_frame
            box_marker.ns = 'semantic_boxes'
            box_marker.id = lid * 2
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose.position.x = lm['x']
            box_marker.pose.position.y = lm['y']
            box_marker.pose.position.z = max(0.2, lm['z'])
            box_marker.scale.x = 0.45
            box_marker.scale.y = 0.45
            box_marker.scale.z = 0.45
            box_marker.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.6)  # Electric Blue
            markers.markers.append(box_marker)

            # Marker B: Floating 3D Text Label
            text_marker = Marker()
            text_marker.header.stamp = now
            text_marker.header.frame_id = self.map_frame
            text_marker.ns = 'semantic_labels'
            text_marker.id = lid * 2 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = lm['x']
            text_marker.pose.position.y = lm['y']
            text_marker.pose.position.z = max(0.2, lm['z']) + 0.35
            text_marker.scale.z = 0.18
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"{lm['label']} ({int(lm['score']*100)}%)"
            markers.markers.append(text_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticObjectMapperNode()
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
