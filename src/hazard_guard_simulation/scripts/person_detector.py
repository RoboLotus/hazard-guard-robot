#!/usr/bin/env python3
"""Detect people in the RGB stream with a COCO-pretrained YOLO11n.

Publishes a Bool the navigation side can gate on later, and an annotated
image so the detection can be checked by eye in rqt_image_view.

Inference is slower than the 30 Hz camera, so frames that arrive while a
frame is in flight are dropped rather than queued - a stale detection is
worse than a missed one for anything that brakes the robot.

    ros2 run hazard_guard_simulation person_detector.py \
        --ros-args -p conf:=0.4 -p device:=cpu
"""
from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover - optional deployment dependency
    YOLO = None

# COCO class 0 is "person"; the model is not retrained, so this is fixed.
PERSON_CLASS_ID = 0


def person_boxes(result, conf: float) -> list[tuple[float, float, float, float, float]]:
    """(x1, y1, x2, y2, score) for every person box above conf."""
    boxes = []
    for box in result.boxes:
        if int(box.cls) != PERSON_CLASS_ID:
            continue
        score = float(box.conf)
        if score < conf:
            continue
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
        boxes.append((x1, y1, x2, y2, score))
    return boxes


class PersonDetector(Node):
    def __init__(self) -> None:
        if YOLO is None:
            raise RuntimeError(
                "ultralytics is not installed. Install the project YOLO "
                "runtime before starting the person detector node."
            )
        super().__init__("person_detector")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detected_topic", "/person_detector/detected")
        self.declare_parameter("annotated_topic", "/person_detector/image")
        # yolo11n.pt is downloaded to the working directory on first use.
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("imgsz", 640)
        # "cpu", "0" for the first CUDA device, or "" to let ultralytics pick.
        self.declare_parameter("device", "cpu")
        self.declare_parameter("publish_annotated", True)

        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value) or None
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)

        self.bridge = CvBridge()
        self.busy = False
        self.model = YOLO(str(self.get_parameter("model").value))

        # Default (reliable) QoS matches the gz bridge on the input side and
        # rqt_image_view on the output side.
        self.detected_publisher = self.create_publisher(
            Bool, self.get_parameter("detected_topic").value, 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, self.get_parameter("annotated_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, 1
        )
        self.get_logger().info(
            f"{self.get_parameter('image_topic').value} -> "
            f"{self.get_parameter('detected_topic').value} "
            f"(conf {self.conf:.2f}, device {self.device or 'auto'})"
        )

    def on_image(self, message: Image) -> None:
        if self.busy:
            return
        self.busy = True
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self.model.predict(
                frame,
                imgsz=self.imgsz,
                conf=self.conf,
                device=self.device,
                classes=[PERSON_CLASS_ID],
                verbose=False,
            )[0]
            boxes = person_boxes(result, self.conf)

            self.detected_publisher.publish(Bool(data=bool(boxes)))
            if boxes:
                best = max(box[4] for box in boxes)
                # Throttled: at camera rate an unthrottled log buries every
                # other message in the console.
                self.get_logger().info(
                    f"사람 {len(boxes)}명 (최고 신뢰도 {best:.2f})",
                    throttle_duration_sec=1.0,
                )

            if self.publish_annotated:
                for x1, y1, x2, y2, score in boxes:
                    cv2.rectangle(
                        frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2
                    )
                    cv2.putText(
                        frame,
                        f"person {score:.2f}",
                        (int(x1), max(int(y1) - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        1,
                    )
                out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                out.header = message.header
                self.annotated_publisher.publish(out)
        finally:
            self.busy = False


def main() -> None:
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
