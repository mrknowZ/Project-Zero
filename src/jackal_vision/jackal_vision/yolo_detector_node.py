"""
Project Zero — SAFiR Lab
YOLO Object Detection Node

Subscribes to an RGB camera image topic, runs YOLOv8 inference, and publishes
vision_msgs/Detection2DArray.  An annotated image is also published for
RViz2 visualisation.

Detected classes are configurable via parameters; the default set satisfies the
project requirement of ≥ 3 categories (person, backpack, bicycle).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from std_msgs.msg import Header

import cv2
import numpy as np

# cv_bridge converts between ROS Image messages and OpenCV images.
from cv_bridge import CvBridge


class YoloDetectorNode(Node):
    """ROS2 node that wraps Ultralytics YOLOv8 for real-time detection."""

    def __init__(self):
        super().__init__('yolo_detector')

        # --------------- Parameters ---------------
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('image_topic',
                               '/j100_0000/sensors/camera_0/color/image')
        self.declare_parameter('detection_rate_hz', 5.0)
        self.declare_parameter(
            'target_classes',
            ['person', 'backpack', 'bicycle'],
        )
        self.declare_parameter('device', 'cpu')  # 'cpu' or '0' for GPU

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        rate_hz = self.get_parameter('detection_rate_hz').value
        self.target_classes = self.get_parameter('target_classes').value
        device = self.get_parameter('device').value

        # --------------- YOLO model ---------------
        try:
            import sys
            import os
            venv_site_packages = os.path.expanduser('~/ros2_env/lib/python3.10/site-packages')
            if venv_site_packages not in sys.path and os.path.isdir(venv_site_packages):
                sys.path.insert(0, venv_site_packages)
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            # Warm-up inference (downloads weights on first run)
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
        for cls_name in self.target_classes:
            for idx, name in self.coco_names.items():
                if name == cls_name:
                    self.target_ids.add(idx)
        self.get_logger().info(
            f'Tracking classes: {self.target_classes} -> IDs {self.target_ids}'
        )

        # --------------- ROS I/O ---------------
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos
        )

        self.det_pub = self.create_publisher(
            Detection2DArray, '~/detections', 10
        )
        self.vis_pub = self.create_publisher(
            Image, '~/detections_image', 10
        )

        # Rate-limit inference to avoid overloading CPU
        self._period = 1.0 / rate_hz
        self._last_inference_time = self.get_clock().now()
        self._latest_image_msg = None

        self.timer = self.create_timer(self._period, self._timer_callback)

        self.get_logger().info(
            f'YoloDetectorNode ready — subscribing to {image_topic} '
            f'@ {rate_hz} Hz max inference rate'
        )

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image):
        """Buffer the latest image; inference runs on the timer."""
        self._latest_image_msg = msg

    # ------------------------------------------------------------------
    def _timer_callback(self):
        """Run inference on the latest buffered image."""
        msg = self._latest_image_msg
        if msg is None:
            return

        # Convert ROS Image → OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
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

                # Populate Detection2D
                det = Detection2D()
                det.header = msg.header

                hyp = ObjectHypothesisWithPose()
                # ROS2 Humble vision_msgs: hyp.hypothesis has class_id (str) and score (float)
                hyp.hypothesis.class_id = str(self.coco_names.get(cls_id, cls_id))
                hyp.hypothesis.score = float(conf)
                det.results.append(hyp)

                # Bounding box (center as vision_msgs/Pose2D with position + size)
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.center.theta = 0.0
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                det_array.detections.append(det)

                # Draw on annotated image
                label = f'{self.coco_names.get(cls_id, cls_id)} {conf:.2f}'
                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated, label,
                    (int(x1), int(y1) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )

        # Publish detections
        self.det_pub.publish(det_array)

        # Publish annotated image
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            vis_msg.header = msg.header
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

        # Clear buffered image so we don't re-process it
        self._latest_image_msg = None


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
