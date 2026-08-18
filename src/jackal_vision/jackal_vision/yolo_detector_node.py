"""
Project Zero — SAFiR Lab
YOLO Object Detection Node

Subscribes to an RGB camera image topic, runs YOLOv8 inference, and publishes
vision_msgs/Detection2DArray. An annotated image is also published for
RViz2 visualisation with color-coded bounding boxes.

Supports common objects: person, bicycle, tree / plant, car, chair, bottle, etc.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from std_msgs.msg import Header

import cv2
import numpy as np


# Common aliases mapped to COCO class names
CLASS_SYNONYMS = {
    'tree': 'potted plant',
    'trees': 'potted plant',
    'plant': 'potted plant',
    'plants': 'potted plant',
    'bike': 'bicycle',
    'motorbike': 'motorcycle',
    'auto': 'car',
    'automobile': 'car',
    'bag': 'backpack',
}

# Color palette for different object types (BGR)
COLOR_PALETTE = [
    (0, 255, 0),     # Bright Green
    (255, 140, 0),   # Deep Sky Blue
    (0, 165, 255),   # Orange
    (238, 130, 238), # Violet
    (0, 215, 255),   # Gold
    (255, 0, 255),   # Magenta
    (0, 255, 255),   # Yellow
    (144, 238, 144), # Light Green
]


def imgmsg_to_cv2(img_msg: Image) -> np.ndarray:
    """Robust conversion of ROS Image message to OpenCV BGR numpy array."""
    if img_msg.encoding == 'bgr8':
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
        return im.copy()
    elif img_msg.encoding == 'rgb8':
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
        return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding in ['mono8', '8UC1']:
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width))
        return cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    else:
        # Fallback raw byte reshape
        im = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, -1))
        if im.shape[2] == 3:
            return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        return im.copy()


def cv2_to_imgmsg(cv_image: np.ndarray, encoding: str = 'bgr8', header: Header = None) -> Image:
    """Robust conversion of OpenCV BGR numpy array to ROS Image message."""
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if len(cv_image.shape) == 2 else cv_image.shape[2]
    msg.step = cv_image.shape[1] * channels
    msg.data = cv_image.tobytes()
    return msg


class YoloDetectorNode(Node):
    """ROS2 node that wraps Ultralytics YOLOv8 for real-time detection."""

    def __init__(self):
        super().__init__('yolo_detector')

        # --------------- Parameters ---------------
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('image_topic',
                               '/j100_0000/sensors/camera_0/color/image')
        self.declare_parameter('detection_rate_hz', 5.0)
        self.declare_parameter(
            'target_classes',
            [
                'person', 'bicycle', 'tree', 'potted plant', 'car',
                'truck', 'bus', 'chair', 'bench', 'bottle', 'backpack'
            ],
        )
        self.declare_parameter('device', 'cpu')  # 'cpu' or '0' for GPU

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        rate_hz = self.get_parameter('detection_rate_hz').value
        raw_target_classes = self.get_parameter('target_classes').value
        # --------------- Device Selection (RTX 3070 GPU vs Intel CPU) ---------------
        try:
            import torch
            if device in ['auto', '0', 'cuda'] and torch.cuda.is_available():
                device = '0'
                gpu_name = torch.cuda.get_device_name(0)
                self.get_logger().info(f'Hardware Acceleration: Utilizing Dedicated GPU [{gpu_name}]')
            else:
                device = 'cpu'
                self.get_logger().info('Hardware Acceleration: Utilizing CPU inference')
        except Exception:
            device = 'cpu'

        # --------------- YOLO model ---------------
        try:
            import sys
            import os
            venv_site_packages = os.path.expanduser('~/ros2_env/lib/python3.10/site-packages')
            if venv_site_packages not in sys.path and os.path.isdir(venv_site_packages):
                sys.path.insert(0, venv_site_packages)
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            # Warm-up inference
            self.model.predict(
                np.zeros((480, 640, 3), dtype=np.uint8),
                device=device,
                verbose=False,
            )
            self.get_logger().info(
                f'YOLOv8 model loaded: {model_path}  device={device}'
            )
        except ImportError:
            self.get_logger().error(
                'ultralytics is not installed. '
                'Install with: pip install ultralytics'
            )
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        self.device = device

        # Build a set of target class indices from COCO names
        self.coco_names = self.model.names  # dict {int: str}
        self.target_ids = set()

        # Resolve synonyms and match against COCO names
        normalized_targets = []
        for cls_name in raw_target_classes:
            name_lower = cls_name.lower().strip()
            resolved = CLASS_SYNONYMS.get(name_lower, name_lower)
            normalized_targets.append(resolved)

        if not normalized_targets or 'all' in normalized_targets:
            self.target_ids = set(self.coco_names.keys())
            self.get_logger().info('Tracking ALL 80 COCO object classes.')
        else:
            for cls_name in normalized_targets:
                for idx, name in self.coco_names.items():
                    if name.lower() == cls_name:
                        self.target_ids.add(idx)
            self.get_logger().info(
                f'Tracking classes: {raw_target_classes} -> IDs {self.target_ids}'
            )

        # --------------- ROS I/O ---------------
        self.det_pub = self.create_publisher(
            Detection2DArray, '~/detections', 10
        )
        self.vis_pub = self.create_publisher(
            Image, '~/detections_image', 10
        )

        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )

        # Rate-limit inference (wall-clock throttling)
        self._period = 1.0 / max(rate_hz, 0.1)
        self._last_infer_time = 0.0

        self.get_logger().info(
            f'YoloDetectorNode ready — subscribing to {image_topic} '
            f'@ {rate_hz} Hz max inference rate'
        )

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image):
        """Process image with YOLOv8 and publish detections and annotated image."""
        now = time.monotonic()
        if (now - self._last_infer_time) < self._period:
            return
        self._last_infer_time = now

        # Convert ROS Image → OpenCV BGR
        try:
            cv_image = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        # Run YOLOv8 inference
        results = self.model.predict(
            cv_image,
            conf=self.conf_thresh,
            device=self.device,
            verbose=False,
            classes=list(self.target_ids) if self.target_ids else None,
        )

        # Build Detection2DArray
        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = cv_image.copy()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cls_name = self.coco_names.get(cls_id, str(cls_id))
                display_name = 'tree/plant' if cls_name == 'potted plant' else cls_name

                # Populate Detection2D
                det = Detection2D()
                det.header = msg.header

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = cls_name
                hyp.hypothesis.score = float(conf)
                det.results.append(hyp)

                # Bounding box
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.center.theta = 0.0
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                det_array.detections.append(det)

                # Select box color
                color = COLOR_PALETTE[cls_id % len(COLOR_PALETTE)]

                # Draw on annotated image
                label = f'{display_name} {conf:.2f}'
                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    color,
                    2,
                )
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(
                    annotated,
                    (int(x1), max(0, int(y1) - 20)),
                    (int(x1) + w + 4, max(20, int(y1))),
                    color,
                    -1,
                )
                cv2.putText(
                    annotated, label,
                    (int(x1) + 2, max(15, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
                    cv2.LINE_AA,
                )

        # Publish detections
        self.det_pub.publish(det_array)

        # Publish annotated image
        try:
            vis_msg = cv2_to_imgmsg(annotated, encoding='bgr8', header=msg.header)
            self.vis_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish annotated image: {e}')

        if det_array.detections:
            classes_found = [
                d.results[0].hypothesis.class_id
                for d in det_array.detections
            ]
            self.get_logger().info(
                f'Detected {len(det_array.detections)} objects: {classes_found}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
