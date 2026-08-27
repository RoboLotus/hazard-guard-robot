#!/usr/bin/env python3
"""Read-only visual validation of physical RGB-D to thermal calibration.

The node subscribes only. It publishes no topic or TF and writes nothing unless
the operator presses S for a screenshot. RGB and depth image centres are
back-projected at the measured centre depth, transformed with the saved
calibration, and drawn on the thermal image.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


DEFAULT_ROOT = Path("~/.local/share/hazard_guard/calibration").expanduser()
WINDOW_NAME = "HazardGuard Calibration Validation (read-only)"
# Three panels are displayed side by side.  Keep the composite inside a
# 1440 px-wide Jetson desktop instead of opening an 1800 px-wide window.
PANEL_WIDTH = 480
PANEL_HEIGHT = 360


def normalize_u8(values: np.ndarray) -> np.ndarray:
    source = values.astype(np.float32)
    finite = np.isfinite(source)
    if not np.any(finite):
        return np.zeros(source.shape, dtype=np.uint8)
    low, high = np.percentile(source[finite], [1.0, 99.0])
    if high - low < 1.0e-6:
        return np.zeros(source.shape, dtype=np.uint8)
    result = np.zeros(source.shape, dtype=np.uint8)
    result[finite] = (
        np.clip((source[finite] - low) / (high - low), 0.0, 1.0) * 255.0
    ).astype(np.uint8)
    return result


def depth_in_metres(depth: np.ndarray, encoding: str) -> np.ndarray:
    if encoding.upper() in {"16UC1", "MONO16"}:
        scale = 0.001
    elif encoding.upper() == "32FC1":
        scale = 1.0
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    return depth.astype(np.float32) * scale


def quaternion_rotation(values: dict) -> np.ndarray:
    x = float(values["x"])
    y = float(values["y"])
    z = float(values["z"])
    w = float(values["w"])
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("invalid extrinsic quaternion")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def load_thermal_intrinsic(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    matrix = np.asarray(document["camera_matrix"]["data"], dtype=np.float64)
    distortion = np.asarray(
        document["distortion_coefficients"]["data"], dtype=np.float64
    )
    size = (int(document["image_width"]), int(document["image_height"]))
    return matrix.reshape(3, 3), distortion.reshape(-1, 1), size


def load_thermal_from_rgb(path: Path) -> tuple[np.ndarray, np.ndarray, str, str]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    parent = str(document["parent_frame_id"])
    child = str(document["child_frame_id"])
    measurement = document.get("measurement", {})
    if measurement.get("direction") == "thermal_from_rgb":
        rotation = np.asarray(
            measurement["rotation_matrix"], dtype=np.float64
        ).reshape(3, 3)
        translation = np.asarray(
            measurement["translation_m"], dtype=np.float64
        ).reshape(3)
        return rotation, translation, parent, child

    # The ROS transform stores the child pose in its parent. Invert it to
    # obtain X_thermal = R * X_rgb + t for direct projection.
    parent_from_child = quaternion_rotation(document["rotation_xyzw"])
    parent_translation = np.array([
        float(document["translation"][axis]) for axis in ("x", "y", "z")
    ])
    thermal_from_rgb = parent_from_child.T
    thermal_translation = -thermal_from_rgb @ parent_translation
    return thermal_from_rgb, thermal_translation, parent, child


def camera_values(info: CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64).reshape(-1, 1)
    if not distortion.size:
        distortion = np.zeros((5, 1), dtype=np.float64)
    return matrix, distortion


def image_centre_ray(
    width: int,
    height: int,
    matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    pixel = np.array([[[0.5 * (width - 1), 0.5 * (height - 1)]]], dtype=np.float64)
    normalized = cv2.undistortPoints(pixel, matrix, distortion).reshape(2)
    return np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)


def median_centre_depth(depth_m: np.ndarray, radius: int = 8) -> float | None:
    height, width = depth_m.shape[:2]
    x = width // 2
    y = height // 2
    region = depth_m[
        max(0, y - radius):min(height, y + radius + 1),
        max(0, x - radius):min(width, x + radius + 1),
    ]
    valid = region[np.isfinite(region) & (region > 0.1)]
    return float(np.median(valid)) if valid.size else None


def project_point(
    point_rgb: np.ndarray,
    thermal_from_rgb: np.ndarray,
    thermal_translation: np.ndarray,
    thermal_matrix: np.ndarray,
    thermal_distortion: np.ndarray,
) -> tuple[float, float] | None:
    point_thermal = thermal_from_rgb @ point_rgb + thermal_translation
    if point_thermal[2] <= 0.0:
        return None
    pixels, _ = cv2.projectPoints(
        point_thermal.reshape(1, 1, 3),
        np.zeros(3), np.zeros(3),
        thermal_matrix, thermal_distortion,
    )
    x, y = pixels.reshape(2)
    return float(x), float(y)


def panel(image: np.ndarray, title: str, detail: str) -> np.ndarray:
    canvas = np.full((PANEL_HEIGHT, PANEL_WIDTH, 3), 22, dtype=np.uint8)
    available = PANEL_HEIGHT - 55
    scale = min(PANEL_WIDTH / image.shape[1], available / image.shape[0])
    resized = cv2.resize(
        image, None, fx=scale, fy=scale,
        interpolation=cv2.INTER_NEAREST if scale > 1.0 else cv2.INTER_AREA,
    )
    x = (PANEL_WIDTH - resized.shape[1]) // 2
    y = 30 + (available - resized.shape[0]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    cv2.putText(
        canvas, title, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, detail, (10, PANEL_HEIGHT - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (235, 235, 235), 1,
        cv2.LINE_AA,
    )
    return canvas


def draw_source_centre(image: np.ndarray, color: tuple[int, int, int]) -> None:
    centre = (image.shape[1] // 2, image.shape[0] // 2)
    cv2.drawMarker(image, centre, color, cv2.MARKER_CROSS, 22, 2)


def draw_thermal_target(
    image: np.ndarray,
    pixel: tuple[float, float] | None,
    color: tuple[int, int, int],
    label: str,
    radius: int,
) -> bool:
    if pixel is None:
        return False
    x, y = int(round(pixel[0])), int(round(pixel[1]))
    if not (0 <= x < image.shape[1] and 0 <= y < image.shape[0]):
        return False
    cv2.circle(image, (x, y), radius, color, 2, cv2.LINE_AA)
    cv2.drawMarker(image, (x, y), color, cv2.MARKER_CROSS, radius * 2, 1)
    cv2.putText(
        image, label, (x + radius + 2, max(12, y - radius)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA,
    )
    return True


class CalibrationValidator(Node):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__("physical_calibration_validator")
        self.arguments = arguments
        self.bridge = CvBridge()
        self.rgb: deque[tuple[float, Image]] = deque(maxlen=10)
        self.depth: deque[tuple[float, Image]] = deque(maxlen=20)
        self.thermal: deque[tuple[float, Image]] = deque(maxlen=10)
        self.rgb_info: CameraInfo | None = None
        self.depth_info: CameraInfo | None = None
        self.last_render = 0.0
        self.last_composite: np.ndarray | None = None
        self.window_opened = False

        self.thermal_k, self.thermal_d, self.thermal_size = load_thermal_intrinsic(
            arguments.intrinsic_file
        )
        (
            self.thermal_from_rgb,
            self.thermal_translation,
            self.rgb_frame,
            self.thermal_frame,
        ) = load_thermal_from_rgb(arguments.extrinsic_file)
        self.report = self._load_report(arguments.report_file)

        self.create_subscription(
            Image, arguments.rgb_topic,
            lambda message: self.rgb.append((time.monotonic(), message)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, arguments.depth_topic,
            lambda message: self.depth.append((time.monotonic(), message)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, arguments.thermal_topic,
            lambda message: self.thermal.append((time.monotonic(), message)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, arguments.rgb_info_topic,
            lambda message: setattr(self, "rgb_info", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, arguments.depth_info_topic,
            lambda message: setattr(self, "depth_info", message),
            qos_profile_sensor_data,
        )

    @staticmethod
    def _load_report(path: Path) -> dict:
        if not path.is_file():
            return {}
        return json.loads(path.read_text(encoding="utf-8"))

    def _latest_triplet(self):
        if not self.rgb or not self.depth or not self.thermal:
            return None
        thermal_receipt, thermal = self.thermal[-1]
        _, rgb = min(self.rgb, key=lambda item: abs(item[0] - thermal_receipt))
        _, depth = min(self.depth, key=lambda item: abs(item[0] - thermal_receipt))
        return rgb, depth, thermal

    def render(self) -> str | None:
        now = time.monotonic()
        if now - self.last_render < 0.10:
            return self._key()
        self.last_render = now
        triplet = self._latest_triplet()
        if triplet is None or self.rgb_info is None or self.depth_info is None:
            waiting = np.zeros((240, 320, 3), dtype=np.uint8)
            body = np.hstack([
                panel(waiting, "RGB", "waiting for image/camera_info"),
                panel(waiting, "DEPTH", "waiting for image/camera_info"),
                panel(waiting, "THERMAL", "waiting for image"),
            ])
            self._show(body, "Waiting for physical camera topics")
            return self._key()

        rgb_message, depth_message, thermal_message = triplet
        rgb = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(
            depth_message, desired_encoding="passthrough"
        )
        thermal_raw = self.bridge.imgmsg_to_cv2(
            thermal_message, desired_encoding="passthrough"
        )
        try:
            depth_m = depth_in_metres(depth_raw, depth_message.encoding)
        except ValueError as error:
            self._show_error(str(error))
            return self._key()

        rgb_view = rgb.copy()
        draw_source_centre(rgb_view, (255, 0, 255))
        depth_gray = normalize_u8(depth_m)
        depth_view = cv2.applyColorMap(depth_gray, cv2.COLORMAP_TURBO)
        depth_view[~(np.isfinite(depth_m) & (depth_m > 0.0))] = 0
        draw_source_centre(depth_view, (255, 255, 0))
        thermal_view = cv2.applyColorMap(
            normalize_u8(thermal_raw), cv2.COLORMAP_INFERNO
        )

        measured_depth = median_centre_depth(depth_m)
        projection_depth = measured_depth or self.arguments.fallback_distance_m
        rgb_k, rgb_d = camera_values(self.rgb_info)
        depth_k, depth_d = camera_values(self.depth_info)
        rgb_point = image_centre_ray(
            rgb.shape[1], rgb.shape[0], rgb_k, rgb_d
        ) * projection_depth
        depth_point = image_centre_ray(
            depth_m.shape[1], depth_m.shape[0], depth_k, depth_d
        ) * projection_depth
        rgb_target = project_point(
            rgb_point, self.thermal_from_rgb, self.thermal_translation,
            self.thermal_k, self.thermal_d,
        )
        depth_target = project_point(
            depth_point, self.thermal_from_rgb, self.thermal_translation,
            self.thermal_k, self.thermal_d,
        )
        rgb_visible = draw_thermal_target(
            thermal_view, rgb_target, (255, 0, 255), "R", 7
        )
        depth_visible = draw_thermal_target(
            thermal_view, depth_target, (255, 255, 0), "D", 11
        )

        source = "measured" if measured_depth is not None else "fallback"
        thermal_detail = (
            f"R magenta / D cyan | z={projection_depth:.2f}m {source} | "
            f"in-frame={'yes' if rgb_visible and depth_visible else 'no'}"
        )
        rgb_detail = f"centre | frame={rgb_message.header.frame_id or '?'}"
        depth_detail = (
            f"centre depth={measured_depth:.2f}m"
            if measured_depth is not None else "centre depth invalid"
        )
        body = np.hstack([
            panel(rgb_view, "RGB", rgb_detail),
            panel(depth_view, "DEPTH", depth_detail),
            panel(thermal_view, "THERMAL + PROJECTED CENTRES", thermal_detail),
        ])
        rms = "RMS unavailable"
        if self.report:
            rms = (
                f"thermal RMS {self.report.get('thermal_rms_px', float('nan')):.3f}px | "
                f"stereo RMS {self.report.get('stereo_rms_px', float('nan')):.3f}px"
            )
        frame_warning = ""
        if rgb_message.header.frame_id != self.rgb_frame:
            frame_warning = " | WARNING: RGB frame differs from calibration"
        self._show(body, rms + frame_warning)
        return self._key()

    def _show(self, body: np.ndarray, status: str) -> None:
        footer = np.full((58, body.shape[1], 3), 18, dtype=np.uint8)
        cv2.putText(
            footer, f"Q quit | S screenshot     {status}",
            (14, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.60,
            (245, 245, 245), 1, cv2.LINE_AA,
        )
        self.last_composite = np.vstack((body, footer))
        if not self.window_opened:
            cv2.namedWindow(
                WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
            )
            cv2.resizeWindow(
                WINDOW_NAME,
                self.last_composite.shape[1],
                self.last_composite.shape[0],
            )
            self.window_opened = True
        cv2.imshow(WINDOW_NAME, self.last_composite)

    def _show_error(self, message: str) -> None:
        image = np.zeros((PANEL_HEIGHT, PANEL_WIDTH * 3, 3), dtype=np.uint8)
        cv2.putText(
            image, message, (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 0, 255), 2, cv2.LINE_AA,
        )
        self._show(image, "Input error")

    @staticmethod
    def _key() -> str | None:
        code = cv2.waitKey(1) & 0xFF
        if code in (255,):
            return None
        if code == 27:
            return "q"
        return chr(code).lower()

    def save_screenshot(self) -> None:
        if self.last_composite is None:
            return
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = Path.cwd() / f"calibration-validation-{stamp}.png"
        cv2.imwrite(str(path), self.last_composite)
        print(f"saved {path}")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--rgb-topic", default="/ascamera_hp60c/camera_publisher/rgb0/image"
    )
    result.add_argument(
        "--rgb-info-topic",
        default="/ascamera_hp60c/camera_publisher/rgb0/camera_info",
    )
    result.add_argument(
        "--depth-topic",
        default="/ascamera_hp60c/camera_publisher/depth0/image_raw",
    )
    result.add_argument(
        "--depth-info-topic",
        default="/ascamera_hp60c/camera_publisher/depth0/camera_info",
    )
    result.add_argument("--thermal-topic", default="/thermal_camera/image_raw")
    result.add_argument(
        "--intrinsic-file",
        type=Path,
        default=DEFAULT_ROOT / "thermal_intrinsics.yaml",
    )
    result.add_argument(
        "--extrinsic-file",
        type=Path,
        default=DEFAULT_ROOT / "thermal_rgb_extrinsic.yaml",
    )
    result.add_argument(
        "--report-file", type=Path,
        default=DEFAULT_ROOT / "latest_report.json",
    )
    result.add_argument("--fallback-distance-m", type=float, default=1.0)
    return result


def main(argv: list[str] | None = None) -> int:
    arguments = parser().parse_args(argv)
    for name in ("intrinsic_file", "extrinsic_file", "report_file"):
        setattr(arguments, name, getattr(arguments, name).expanduser().resolve())
    for path in (arguments.intrinsic_file, arguments.extrinsic_file):
        if not path.is_file():
            raise SystemExit(f"calibration file not found: {path}")
    if arguments.fallback_distance_m <= 0.0:
        raise SystemExit("fallback-distance-m must be positive")

    rclpy.init()
    node = CalibrationValidator(arguments)
    print("Read-only RGB-D / thermal calibration validation")
    print("Magenta R = RGB centre, cyan D = depth centre, Q = quit, S = screenshot")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = node.render()
            if key == "q":
                return 0
            if key == "s":
                node.save_screenshot()
    except KeyboardInterrupt:
        return 130
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
