#!/usr/bin/env python3
"""Publish a ThermoEye TMC160F as ROS 2 image and camera-info topics.

TmSDK exposes Y16 samples as Kelvin x 100.  The canonical ``image_raw`` topic
keeps that representation so existing HazardGuard consumers can convert it
with ``temperature_c = sample * 0.01 - 273.15``.  ``image_sensor_raw`` keeps a
separate copy of the SDK samples so the source observation is not lost if the
canonical representation changes later.
"""

from __future__ import annotations

import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml

try:
    from TmCore import TmCamera
    from TmCore.TmTypes import ColorOrder, ColormapTypes
except ImportError:  # pragma: no cover - depends on the vendor install
    TmCamera = None
    ColorOrder = None
    ColormapTypes = None


KELVIN_PER_COUNT = 0.01
ABSOLUTE_ZERO_C = -273.15


def sdk_pixels_to_image(values: object, width: int, height: int) -> np.ndarray:
    """Convert TmSDK's width-major pixel matrix to a ROS row-major image."""
    pixels = np.asarray(values)
    if pixels.shape == (width, height):
        pixels = pixels.T
    elif pixels.shape != (height, width):
        raise ValueError(
            f"unexpected TmSDK pixel shape {pixels.shape}; "
            f"expected {(width, height)} or {(height, width)}"
        )
    return np.clip(pixels, 0, np.iinfo(np.uint16).max).astype(
        np.uint16, copy=False
    )


def estimated_camera_info(
    width: int, height: int, frame_id: str, horizontal_fov_deg: float
) -> CameraInfo:
    """Build provisional intrinsics until a physical calibration is supplied."""
    focal = (width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)
    cx = width / 2.0
    cy = height / 2.0
    message = CameraInfo()
    message.header.frame_id = frame_id
    message.width = width
    message.height = height
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [
        focal, 0.0, cx, 0.0,
        0.0, focal, cy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def calibrated_camera_info(path: Path, frame_id: str) -> CameraInfo:
    """Load a standard ROS camera-calibration YAML file."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"camera calibration must be a mapping: {path}")

    def matrix(name: str, expected: int) -> list[float]:
        item = document.get(name)
        data = item.get("data") if isinstance(item, dict) else None
        if not isinstance(data, list) or len(data) != expected:
            raise ValueError(
                f"{name}.data must contain {expected} values in {path}"
            )
        return [float(value) for value in data]

    message = CameraInfo()
    message.header.frame_id = frame_id
    message.width = int(document["image_width"])
    message.height = int(document["image_height"])
    message.distortion_model = str(document.get("distortion_model", "plumb_bob"))
    distortion = document.get("distortion_coefficients")
    coefficients = distortion.get("data") if isinstance(distortion, dict) else None
    if not isinstance(coefficients, list):
        raise ValueError(f"distortion_coefficients.data is missing in {path}")
    message.d = [float(value) for value in coefficients]
    message.k = matrix("camera_matrix", 9)
    message.r = matrix("rectification_matrix", 9)
    message.p = matrix("projection_matrix", 12)
    return message


def calibrated_extrinsic(path: Path, expected_child: str) -> TransformStamped:
    """Load the physical RGB-parent to thermal-child static transform YAML."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"thermal extrinsic must be a mapping: {path}")
    parent = str(document.get("parent_frame_id", "")).strip()
    child = str(document.get("child_frame_id", "")).strip()
    if not parent or not child:
        raise ValueError(f"parent_frame_id and child_frame_id are required: {path}")
    if child != expected_child:
        raise ValueError(
            f"extrinsic child frame {child!r} does not match configured "
            f"frame_id {expected_child!r}"
        )
    translation = document.get("translation")
    rotation = document.get("rotation_xyzw")
    if not isinstance(translation, dict) or not isinstance(rotation, dict):
        raise ValueError(f"translation and rotation_xyzw are required: {path}")

    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(translation["x"])
    message.transform.translation.y = float(translation["y"])
    message.transform.translation.z = float(translation["z"])
    message.transform.rotation.x = float(rotation["x"])
    message.transform.rotation.y = float(rotation["y"])
    message.transform.rotation.z = float(rotation["z"])
    message.transform.rotation.w = float(rotation["w"])
    norm = math.sqrt(
        message.transform.rotation.x ** 2
        + message.transform.rotation.y ** 2
        + message.transform.rotation.z ** 2
        + message.transform.rotation.w ** 2
    )
    if not math.isfinite(norm) or abs(norm - 1.0) > 1.0e-3:
        raise ValueError(f"extrinsic quaternion is not normalized: {path}")
    return message


def image_message(pixels: np.ndarray, encoding: str, frame_id: str, stamp) -> Image:
    pixels = np.ascontiguousarray(pixels)
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height, message.width = pixels.shape[:2]
    message.encoding = encoding
    message.is_bigendian = False
    message.step = int(pixels.strides[0])
    message.data = pixels.tobytes()
    return message


def sdk_bitmap_to_bgr(bitmap: bytes, width: int, height: int) -> np.ndarray:
    """Convert TmSDK RGB bitmap bytes to a ROS/OpenCV BGR image."""
    rgb = np.frombuffer(bitmap, dtype=np.uint8)
    expected = width * height * 3
    if rgb.size != expected:
        raise ValueError(
            f"unexpected TmSDK bitmap size {rgb.size}; expected {expected}"
        )
    return np.ascontiguousarray(rgb.reshape(height, width, 3)[:, :, ::-1])


def color_scale_bounds(
    celsius: np.ndarray,
    mode: str,
    fixed_low_c: float,
    fixed_high_c: float,
    relative_low_percentile: float,
    relative_high_percentile: float,
) -> tuple[float, float]:
    """Return robust display bounds without changing radiometric samples."""
    if mode == "fixed":
        return fixed_low_c, fixed_high_c
    if mode != "relative":
        raise ValueError("color_scale_mode must be 'relative' or 'fixed'")
    if not 0.0 <= relative_low_percentile < relative_high_percentile <= 100.0:
        raise ValueError(
            "relative color percentiles must satisfy "
            "0 <= low < high <= 100"
        )
    low, high = np.percentile(
        celsius, [relative_low_percentile, relative_high_percentile]
    )
    return float(low), float(high)


class ThermalCameraPublisher(Node):
    def __init__(self) -> None:
        if TmCamera is None or ColorOrder is None or ColormapTypes is None:
            raise RuntimeError(
                "TmSDK Python bindings are not installed. Install the "
                "ThermoEye ARM64 native package and Python wheel before "
                "starting this node."
            )
        super().__init__("thermal_camera_publisher")
        self.declare_parameter("model", "TMC160F")
        self.declare_parameter("frame_id", "thermal_camera_optical_frame")
        self.declare_parameter("width", 160)
        self.declare_parameter("height", 120)
        self.declare_parameter("publish_rate_hz", 9.0)
        self.declare_parameter("horizontal_fov_deg", 57.0)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("extrinsic_file", "")
        self.declare_parameter("min_temp_c", 10.0)
        self.declare_parameter("max_temp_c", 60.0)
        self.declare_parameter("color_scale_mode", "sdk")
        self.declare_parameter("relative_low_percentile", 2.0)
        self.declare_parameter("relative_high_percentile", 98.0)
        self.declare_parameter("sdk_noise_filtering", True)
        self.declare_parameter("reconnect_interval_sec", 2.0)

        self._camera = TmCamera()
        self._connected = False
        self._last_connect_attempt = -math.inf
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._camera_info = self._make_camera_info()
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_extrinsic()

        self._raw_publisher = self.create_publisher(
            Image, "/thermal_camera/image_raw", qos_profile_sensor_data
        )
        self._sensor_raw_publisher = self.create_publisher(
            Image, "/thermal_camera/image_sensor_raw", qos_profile_sensor_data
        )
        self._color_publisher = self.create_publisher(
            Image, "/thermal_camera/image_color", qos_profile_sensor_data
        )
        self._info_publisher = self.create_publisher(
            CameraInfo, "/thermal_camera/camera_info", qos_profile_sensor_data
        )

        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._capture)
        self.get_logger().info(
            "ThermoEye publisher configured for /thermal_camera/image_raw, "
            "/thermal_camera/image_sensor_raw, /thermal_camera/image_color, "
            "and /thermal_camera/camera_info"
        )

    def _make_camera_info(self) -> CameraInfo:
        value = str(self.get_parameter("calibration_file").value).strip()
        if value:
            path = Path(value).expanduser().resolve()
            if path.is_file():
                message = calibrated_camera_info(path, self._frame_id)
                if (message.width, message.height) != (self._width, self._height):
                    raise ValueError(
                        "calibration resolution does not match requested camera "
                        f"resolution: {(message.width, message.height)} != "
                        f"{(self._width, self._height)}"
                    )
                self.get_logger().info(f"Loaded thermal calibration: {path}")
                return message
            self.get_logger().warning(
                f"Thermal calibration file does not exist yet: {path}"
            )
        self.get_logger().warning(
            "No physical thermal calibration file was supplied; CameraInfo "
            "uses the provisional 57-degree FOV and zero distortion"
        )
        return estimated_camera_info(
            self._width,
            self._height,
            self._frame_id,
            float(self.get_parameter("horizontal_fov_deg").value),
        )

    def _publish_extrinsic(self) -> None:
        value = str(self.get_parameter("extrinsic_file").value).strip()
        if not value:
            self.get_logger().warning(
                "No physical thermal extrinsic file was supplied; thermal "
                "frame is not attached to the RGB-D TF tree"
            )
            return
        path = Path(value).expanduser().resolve()
        if not path.is_file():
            self.get_logger().warning(
                f"Thermal extrinsic file does not exist yet: {path}"
            )
            return
        message = calibrated_extrinsic(path, self._frame_id)
        message.header.stamp = self.get_clock().now().to_msg()
        self._static_broadcaster.sendTransform(message)
        self.get_logger().info(
            f"Loaded thermal extrinsic: {message.header.frame_id} -> "
            f"{message.child_frame_id} ({path})"
        )

    def _connect(self) -> bool:
        now = time.monotonic()
        retry = float(self.get_parameter("reconnect_interval_sec").value)
        if now - self._last_connect_attempt < retry:
            return False
        self._last_connect_attempt = now
        requested_model = str(self.get_parameter("model").value)
        try:
            cameras = self._camera.get_local_camera_list()
            selected = next(
                (camera for camera in cameras if camera.name == requested_model), None
            )
            if selected is None:
                names = ", ".join(camera.name for camera in cameras) or "none"
                self.get_logger().warning(
                    f"{requested_model} not found; discovered cameras: {names}",
                    throttle_duration_sec=5.0,
                )
                return False
            formats = [item.format for item in selected.media_info_list]
            if "Y16" not in formats:
                raise RuntimeError(f"camera does not expose Y16: {formats}")
            if not self._camera.open_local_camera(
                selected.name, selected.com_port, selected.index, "Y16"
            ):
                raise RuntimeError(
                    f"failed to open {selected.name} on {selected.com_port}"
                )
            # QueryFrame is the SDK's polling acquisition mode.  Do not call
            # BeginAcquisition here: TmSDK 2.1 reserves it for callback mode,
            # and running both modes makes the polling capture time out.
            self._camera.set_color_map(ColormapTypes.Inferno)
            self._camera.set_noise_filtering(
                bool(self.get_parameter("sdk_noise_filtering").value)
            )
            self._connected = True
            self.get_logger().info(
                f"Opened {selected.name} on {selected.com_port}, "
                f"video index {selected.index}, Y16 {self._width}x{self._height}"
            )
            return True
        except Exception as exc:  # vendor API error boundary
            self.get_logger().error(f"ThermoEye connection failed: {exc}")
            self._disconnect()
            return False

    def _disconnect(self) -> None:
        if self._connected:
            try:
                self._camera.close()
            except Exception:
                pass
        self._connected = False

    def _capture(self) -> None:
        if not self._connected and not self._connect():
            return
        frame = None
        try:
            if not self._camera.is_connected():
                raise RuntimeError("camera disconnected")
            frame = self._camera.query_frame(self._width, self._height)
            if frame is None:
                return
            width = int(frame.width())
            height = int(frame.height())
            if (width, height) != (self._width, self._height):
                raise RuntimeError(
                    f"unexpected frame size {(width, height)}; "
                    f"expected {(self._width, self._height)}"
                )
            raw = sdk_pixels_to_image(
                frame.get_pixel(0, 0, width, height), width, height
            )
            stamp = self.get_clock().now().to_msg()

            # TmSDK Y16 values are Kelvin x 100 on TMC160F.  Keep the two
            # contracts separate even though their bytes are currently equal.
            self._sensor_raw_publisher.publish(
                image_message(raw, "16UC1", self._frame_id, stamp)
            )
            self._raw_publisher.publish(
                image_message(raw, "mono16", self._frame_id, stamp)
            )

            color_mode = str(
                self.get_parameter("color_scale_mode").value
            ).lower()
            if color_mode == "sdk":
                color = sdk_bitmap_to_bgr(
                    frame.to_bitmap(ColorOrder.COLOR_RGB), width, height
                )
            else:
                celsius = (
                    raw.astype(np.float32) * KELVIN_PER_COUNT + ABSOLUTE_ZERO_C
                )
                low, high = color_scale_bounds(
                    celsius,
                    color_mode,
                    float(self.get_parameter("min_temp_c").value),
                    float(self.get_parameter("max_temp_c").value),
                    float(self.get_parameter("relative_low_percentile").value),
                    float(self.get_parameter("relative_high_percentile").value),
                )
                scaled = np.clip(
                    (celsius - low) / max(high - low, 1.0e-3), 0.0, 1.0
                )
                color = cv2.applyColorMap(
                    (scaled * 255.0).astype(np.uint8), cv2.COLORMAP_JET
                )
            self._color_publisher.publish(
                image_message(color, "bgr8", self._frame_id, stamp)
            )
            self._camera_info.header.stamp = stamp
            self._info_publisher.publish(self._camera_info)
        except Exception as exc:  # vendor API error boundary
            self.get_logger().error(
                f"ThermoEye frame capture failed: {exc}",
                throttle_duration_sec=5.0,
            )
            self._disconnect()
        finally:
            if frame is not None:
                try:
                    frame.release()
                except Exception:
                    pass

    def destroy_node(self) -> bool:
        self._disconnect()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ThermalCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
