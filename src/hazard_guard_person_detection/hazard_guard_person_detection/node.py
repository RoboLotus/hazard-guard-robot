"""ROS 2 person detection and RGB-D distance node."""

from collections import deque
from threading import Lock
from typing import Any, Optional, Tuple

import numpy as np

from .backend import UltralyticsBackend
from .detection import PersonDetection, detections_from_ultralytics
from .distance import depth_image_to_metres, estimate_bbox_distance


def _stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def main(args=None) -> None:
    # ROS and cv_bridge are imported only by the executable entry point.  This
    # keeps the pure conversion modules importable on machines without ROS.
    import rclpy
    from cv_bridge import CvBridge
    from hazard_guard_interfaces.msg import PersonObservation, PersonObservationArray
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class PersonDetectionNode(Node):
        def __init__(self) -> None:
            super().__init__("person_detection")
            self._declare_parameters()
            self._bridge = CvBridge()
            self._callback_group = ReentrantCallbackGroup()
            self._lock = Lock()
            self._latest_rgb: Optional[Any] = None
            self._depth_frames: deque[Tuple[Any, np.ndarray]] = deque(maxlen=3)
            self._inference_running = False
            self._frame_sequence = 0

            self._backend = UltralyticsBackend(
                model_path=str(self.get_parameter("model_path").value),
                confidence=float(self.get_parameter("confidence").value),
                image_size=int(self.get_parameter("image_size").value),
                device=str(self.get_parameter("device").value),
                person_class_id=int(self.get_parameter("person_class_id").value),
            )
            self._observations_publisher = self.create_publisher(
                PersonObservationArray,
                str(self.get_parameter("observations_topic").value),
                10,
            )
            self._annotated_publisher = self.create_publisher(
                Image,
                str(self.get_parameter("annotated_image_topic").value),
                2,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("rgb_topic").value),
                self._on_rgb,
                qos_profile_sensor_data,
                callback_group=self._callback_group,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("depth_topic").value),
                self._on_depth,
                qos_profile_sensor_data,
                callback_group=self._callback_group,
            )
            inference_rate_hz = max(0.1, float(self.get_parameter("inference_rate_hz").value))
            self.create_timer(
                1.0 / inference_rate_hz,
                self._run_inference,
                callback_group=self._callback_group,
            )
            self.get_logger().info(
                f"Person detector ready; backend={self._backend.name}, "
                f"rate={inference_rate_hz:.1f} Hz (model loads on first frame)"
            )

        def _declare_parameters(self) -> None:
            defaults = {
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/depth/image_raw",
                "observations_topic": "/hazard_guard/person/observations",
                "annotated_image_topic": "/hazard_guard/person/annotated_image",
                "model_path": "yolo11n.pt",
                "confidence": 0.4,
                "image_size": 640,
                "device": "",
                "person_class_id": 0,
                "inference_rate_hz": 10.0,
                "central_roi_ratio": 0.5,
                "minimum_distance_m": 0.15,
                "maximum_distance_m": 8.0,
                "minimum_valid_depth_samples": 9,
                "maximum_depth_age_sec": 0.10,
                "depth_registration_verified": False,
                "simulated": False,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)

        def _on_rgb(self, message: Any) -> None:
            with self._lock:
                self._latest_rgb = message

        def _on_depth(self, message: Any) -> None:
            try:
                depth = self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                depth_m = depth_image_to_metres(depth, message.encoding)
            except Exception as exc:
                self.get_logger().warning(f"Depth frame conversion failed: {exc}")
                return
            with self._lock:
                self._depth_frames.append((message.header, depth_m))

        def _take_latest_rgb(self) -> Optional[Any]:
            with self._lock:
                if self._inference_running or self._latest_rgb is None:
                    return None
                message = self._latest_rgb
                self._latest_rgb = None
                self._inference_running = True
                return message

        def _depth_for(
            self,
            rgb_header: Any,
            rgb_shape: tuple[int, ...],
        ) -> tuple[Optional[np.ndarray], float, str]:
            if not bool(
                self.get_parameter("depth_registration_verified").value
            ):
                return None, float("inf"), "depth registration is not verified"
            with self._lock:
                depth_frames = tuple(self._depth_frames)
            if not depth_frames:
                return None, float("inf"), "no depth frame received"
            rgb_stamp = _stamp_seconds(rgb_header.stamp)
            depth_header, depth_m = min(
                depth_frames,
                key=lambda value: abs(
                    rgb_stamp - _stamp_seconds(value[0].stamp)
                ),
            )
            maximum_age = float(self.get_parameter("maximum_depth_age_sec").value)
            age = abs(rgb_stamp - _stamp_seconds(depth_header.stamp))
            if age > maximum_age:
                return None, age, "RGB-depth timestamp skew exceeds limit"
            if tuple(rgb_shape[:2]) != tuple(depth_m.shape[:2]):
                return None, age, "RGB-depth image dimensions do not match"
            return depth_m, age, "RGB-depth pair is healthy"

        def _run_inference(self) -> None:
            rgb_message = self._take_latest_rgb()
            if rgb_message is None:
                return
            try:
                rgb = self._bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
                result, inference_ms = self._backend.infer(rgb)
                detections = detections_from_ultralytics(
                    result,
                    person_class_id=int(self.get_parameter("person_class_id").value),
                    minimum_confidence=float(self.get_parameter("confidence").value),
                )
                depth_m, depth_skew_sec, health_reason = self._depth_for(
                    rgb_message.header,
                    rgb.shape,
                )
                self._publish_observations(
                    rgb_message.header,
                    detections,
                    rgb.shape,
                    depth_m,
                    depth_skew_sec,
                    health_reason,
                    inference_ms,
                )
                self._publish_annotated(rgb_message.header, result)
            except Exception as exc:
                self.get_logger().error(f"Person inference failed: {exc}")
            finally:
                with self._lock:
                    self._inference_running = False

        def _publish_observations(
            self,
            header: Any,
            detections: list[PersonDetection],
            rgb_shape: tuple[int, ...],
            depth_m: Optional[np.ndarray],
            depth_skew_sec: float,
            health_reason: str,
            inference_ms: float,
        ) -> None:
            self._frame_sequence += 1
            message = PersonObservationArray()
            message.header = header
            message.inference_ms = float(inference_ms)
            message.backend = self._backend.name
            message.simulated = bool(self.get_parameter("simulated").value)
            message.rgb_valid = True
            message.depth_valid = depth_m is not None
            message.rgb_depth_skew_sec = (
                float(depth_skew_sec) if np.isfinite(depth_skew_sec) else -1.0
            )
            message.rgb_height = int(rgb_shape[0])
            message.rgb_width = int(rgb_shape[1])
            if depth_m is not None:
                message.depth_height = int(depth_m.shape[0])
                message.depth_width = int(depth_m.shape[1])
            message.health_reason = health_reason
            for index, detection in enumerate(detections):
                observation = PersonObservation()
                observation.detection_id = f"person-{self._frame_sequence:08d}-{index:02d}"
                observation.confidence = float(detection.confidence)
                observation.bbox_center_x = int(round(detection.center_x))
                observation.bbox_center_y = int(round(detection.center_y))
                observation.bbox_size_x = int(round(detection.size_x))
                observation.bbox_size_y = int(round(detection.size_y))
                if depth_m is not None:
                    estimate = estimate_bbox_distance(
                        depth_m,
                        (detection.x_min, detection.y_min, detection.x_max, detection.y_max),
                        central_roi_ratio=float(self.get_parameter("central_roi_ratio").value),
                        minimum_distance_m=float(self.get_parameter("minimum_distance_m").value),
                        maximum_distance_m=float(self.get_parameter("maximum_distance_m").value),
                        minimum_valid_samples=int(
                            self.get_parameter("minimum_valid_depth_samples").value
                        ),
                    )
                    observation.distance_m = estimate.distance_m
                    observation.distance_valid = estimate.valid
                else:
                    observation.distance_m = 0.0
                    observation.distance_valid = False
                message.observations.append(observation)
            self._observations_publisher.publish(message)

        def _publish_annotated(self, header: Any, result: Any) -> None:
            if self._annotated_publisher.get_subscription_count() == 0:
                return
            plot = getattr(result, "plot", None)
            if not callable(plot):
                return
            annotated = self._bridge.cv2_to_imgmsg(plot(), encoding="bgr8")
            annotated.header = header
            self._annotated_publisher.publish(annotated)

    rclpy.init(args=args)
    node = PersonDetectionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
