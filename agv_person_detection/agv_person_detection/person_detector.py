#!/usr/bin/env python3
"""
Person detection node with proximity warning.

Subscribes to ifm O3R RGB + depth images, runs YOLOv8 person detection,
and publishes proximity warnings based on depth to detected persons.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import time
import json


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # Declare parameters
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class_id', 0)
        self.declare_parameter('danger_zone', 1.0)
        self.declare_parameter('warning_zone', 2.5)
        self.declare_parameter('caution_zone', 4.0)
        self.declare_parameter('depth_roi_scale', 0.4)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('rgb_topic', '/ifm3d/camera/rgb_2d')
        self.declare_parameter('depth_topic', '/ifm3d/camera/distance')
        self.declare_parameter('depth_scale', 1.0)
        self.declare_parameter('max_detection_rate', 10.0)

        # Get parameters
        model_name = self.get_parameter('model').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.target_class = self.get_parameter('target_class_id').value
        self.danger_dist = self.get_parameter('danger_zone').value
        self.warning_dist = self.get_parameter('warning_zone').value
        self.caution_dist = self.get_parameter('caution_zone').value
        self.depth_roi_scale = self.get_parameter('depth_roi_scale').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.max_rate = self.get_parameter('max_detection_rate').value

        # Load YOLOv8
        self.get_logger().info(f'Loading YOLO model: {model_name}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_name)
            self.get_logger().info('YOLO model loaded successfully')
        except ImportError:
            self.get_logger().fatal(
                'ultralytics not installed! Run: pip install ultralytics'
            )
            raise

        self.bridge = CvBridge()
        self.last_detect_time = 0.0

        # QoS matching the camera (RELIABLE)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Independent subscribers (RGB and depth come from different camera
        # heads at different rates, so time sync is too strict)
        self.rgb_sub = self.create_subscription(
            CompressedImage, rgb_topic, self._rgb_callback, sensor_qos
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._depth_callback, sensor_qos
        )

        self.latest_depth = None
        self.latest_rgb = None

        # Subscribe to camera_info for undistortion
        self.camera_matrix = None
        self.dist_coeffs = None
        self.undistort_map1 = None
        self.undistort_map2 = None
        camera_info_topic = rgb_topic.rsplit('/', 1)[0] + '/camera_info'
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, sensor_qos
        )
        self.get_logger().info(f'Subscribing to camera_info: {camera_info_topic}')

        # Publishers
        self.proximity_pub = self.create_publisher(String, '/person_detection/proximity', 10)
        self.detections_pub = self.create_publisher(String, '/person_detection/detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/person_detection/markers', 10)
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._safety_stop_active = False

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/person_detection/annotated', 1)

        self.get_logger().info(
            f'Person detector ready. Zones: danger<{self.danger_dist}m, '
            f'warning<{self.warning_dist}m, caution<{self.caution_dist}m'
        )

    def _camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return  # Already have calibration
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d)
        if K[0, 0] == 0:
            return  # Invalid
        self.camera_matrix = K
        self.dist_coeffs = D
        # Precompute undistortion maps for speed
        h, w = int(msg.height), int(msg.width)
        new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
        self.undistort_map1, self.undistort_map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (w, h), cv2.CV_16SC2
        )
        self.get_logger().info(
            f'Got camera calibration: fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
            f'cx={K[0,2]:.1f} cy={K[1,2]:.1f} dist={D.tolist()}'
        )

    def _rgb_callback(self, msg: CompressedImage):
        self.latest_rgb = msg
        if self.latest_depth is not None:
            self.detection_callback(self.latest_rgb, self.latest_depth)

    def _depth_callback(self, msg: Image):
        self.latest_depth = msg

    def detection_callback(self, rgb_msg: CompressedImage, depth_msg: Image):
        # Rate limiting
        now = time.time()
        if self.max_rate > 0 and (now - self.last_detect_time) < (1.0 / self.max_rate):
            return
        self.last_detect_time = now

        try:
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb_image is None:
                self.get_logger().error('Failed to decode compressed RGB')
                return
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB: {e}')
            return

        # Undistort using precomputed maps from camera_info
        if self.undistort_map1 is not None:
            rgb_image = cv2.remap(rgb_image, self.undistort_map1, self.undistort_map2, cv2.INTER_LINEAR)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth: {e}')
            return

        # Run YOLO inference (only person class)
        results = self.model(
            rgb_image,
            conf=self.conf_thresh,
            classes=[self.target_class],
            verbose=False
        )

        detections = []
        highest_alert = 'clear'
        closest_distance = float('inf')

        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            h_rgb, w_rgb = rgb_image.shape[:2]
            h_depth, w_depth = depth_image.shape[:2]

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])

                # Map bounding box to depth image coordinates
                scale_x = w_depth / w_rgb
                scale_y = h_depth / h_rgb
                dx1 = int(x1 * scale_x)
                dy1 = int(y1 * scale_y)
                dx2 = int(x2 * scale_x)
                dy2 = int(y2 * scale_y)

                # Extract center ROI of bounding box for robust depth
                cx = (dx1 + dx2) // 2
                cy = (dy1 + dy2) // 2
                roi_w = int((dx2 - dx1) * self.depth_roi_scale) // 2
                roi_h = int((dy2 - dy1) * self.depth_roi_scale) // 2
                roi_x1 = max(0, cx - roi_w)
                roi_y1 = max(0, cy - roi_h)
                roi_x2 = min(w_depth, cx + roi_w)
                roi_y2 = min(h_depth, cy + roi_h)

                depth_roi = depth_image[roi_y1:roi_y2, roi_x1:roi_x2].astype(np.float64)
                depth_roi = depth_roi * self.depth_scale

                # Filter out invalid depths (0, nan, inf)
                valid = depth_roi[(depth_roi > 0.1) & np.isfinite(depth_roi)]
                if len(valid) == 0:
                    distance = -1.0
                else:
                    distance = float(np.median(valid))

                # Classify proximity
                if distance > 0:
                    if distance < self.danger_dist:
                        level = 'DANGER'
                    elif distance < self.warning_dist:
                        level = 'WARNING'
                    elif distance < self.caution_dist:
                        level = 'CAUTION'
                    else:
                        level = 'OK'

                    if distance < closest_distance:
                        closest_distance = distance
                        if level == 'DANGER':
                            highest_alert = 'DANGER'
                        elif level == 'WARNING' and highest_alert not in ('DANGER',):
                            highest_alert = 'WARNING'
                        elif level == 'CAUTION' and highest_alert not in ('DANGER', 'WARNING'):
                            highest_alert = 'CAUTION'
                else:
                    level = 'UNKNOWN'

                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'confidence': round(conf, 2),
                    'distance_m': round(distance, 2) if distance > 0 else None,
                    'proximity': level
                })

        # Publish proximity status
        import json
        proximity_msg = String()
        proximity_msg.data = json.dumps({
            'alert_level': highest_alert,
            'closest_person_m': round(closest_distance, 2) if closest_distance < float('inf') else None,
            'person_count': len(detections),
            'timestamp': rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        })
        self.proximity_pub.publish(proximity_msg)

        # Publish detailed detections
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self.detections_pub.publish(det_msg)

        # Publish RViz markers
        self._publish_markers(detections, rgb_msg.header)

        # Safety auto-stop at ≤1m
        from geometry_msgs.msg import Twist
        if highest_alert == 'DANGER':
            if not self._safety_stop_active:
                self._safety_stop_active = True
                self.get_logger().warn('SAFETY STOP: Person within 1m!')
            stop = Twist()  # all zeros = full stop
            self.cmd_vel_pub.publish(stop)
        else:
            self._safety_stop_active = False

        # Log alerts
        if highest_alert == 'DANGER':
            self.get_logger().warn(
                f'DANGER: Person detected at {closest_distance:.1f}m! '
                f'({len(detections)} person(s))'
            )
        elif highest_alert == 'WARNING':
            self.get_logger().warn(
                f'WARNING: Person at {closest_distance:.1f}m '
                f'({len(detections)} person(s))'
            )

        # Publish annotated image
        if self.publish_annotated and results:
            self._publish_annotated(rgb_image, detections, rgb_msg.header)

    def _publish_annotated(self, image, detections, header):
        import cv2
        annotated = image.copy()

        colors = {
            'DANGER': (0, 0, 255),
            'WARNING': (0, 165, 255),
            'CAUTION': (0, 255, 255),
            'OK': (0, 255, 0),
            'UNKNOWN': (128, 128, 128),
        }

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            level = det['proximity']
            color = colors.get(level, (255, 255, 255))

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            dist_str = f"{det['distance_m']:.1f}m" if det['distance_m'] else "N/A"
            label = f"Person {dist_str} [{level}]"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (x1, y1 - th - 8), (x1 + tw, y1), color, -1)
            cv2.putText(
                annotated, label, (x1, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2
            )

        try:
            msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            msg.header = header
            self.annotated_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')

    def _publish_markers(self, detections, header):
        marker_array = MarkerArray()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, det in enumerate(detections):
            if det['distance_m'] is None:
                continue

            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'camera_optical_link'
            marker.ns = 'person_detection'
            marker.id = i + 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            # Place marker at estimated position (center of bbox projected forward)
            x1, y1, x2, y2 = det['bbox']
            cx_norm = ((x1 + x2) / 2.0 - 640) / 640  # normalized [-1, 1]
            marker.pose.position.x = cx_norm * det['distance_m'] * 0.5
            marker.pose.position.y = 0.0
            marker.pose.position.z = det['distance_m']
            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.3
            marker.text = f"Person: {det['distance_m']:.1f}m"

            level = det['proximity']
            if level == 'DANGER':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            elif level == 'WARNING':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
            elif level == 'CAUTION':
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
