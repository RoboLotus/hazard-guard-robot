"""Calibrate the thermal camera against the RGB camera from the circle target.

Drives the target through a set of poses, captures a synchronised pair at each
one, finds the circle grid in both images, and solves for the transform between
the two cameras. In simulation the answer is already known from the URDF, so
the run ends by scoring itself against it - which is the whole point of doing
this here first. On the real robot the same code runs unchanged; only the
scoring at the end has nothing to compare to.

Two details that are not obvious:

* The circles are found with a blob detector whose shape filters are switched
  off. At 160 x 120 a 70 mm circle is about 11 px across and comes out
  polygonal, and OpenCV's default circularity filter throws those away.
* The thermal image is inverted before detection. Blob detectors look for dark
  blobs, and warm circles are bright.

    python3 tools/calibrate_thermal_rgb.py --poses 20
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

GRID = (4, 4)
SPACING = 0.120
WORLD = "demo_facility_scaled"
TARGET = "cal_target"


def blob_detector(min_area: float):
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = min_area
    params.maxArea = 100000
    params.filterByCircularity = False
    params.filterByInertia = False
    params.filterByConvexity = False
    params.minThreshold = 10
    params.maxThreshold = 220
    params.thresholdStep = 10
    params.minDistBetweenBlobs = 3
    return cv2.SimpleBlobDetector_create(params)


def object_points() -> np.ndarray:
    """Circle centres on the board, in board coordinates (metres)."""
    columns, rows = GRID
    points = np.zeros((rows * columns, 3), np.float32)
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] = grid * SPACING
    return points


def quaternion(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def set_target_pose(x, y, z, roll, pitch, yaw) -> None:
    qx, qy, qz, qw = quaternion(roll, pitch, yaw)
    request = (
        f'name: "{TARGET}", position: {{x: {x}, y: {y}, z: {z}}}, '
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
    )
    subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
         "--timeout", "3000", "--req", request],
        capture_output=True, check=False,
    )


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("calibration_capture")
        self.thermal = self.rgb = None
        self.thermal_info: CameraInfo | None = None
        self.rgb_info: CameraInfo | None = None
        self.create_subscription(Image, "/thermal_camera/image_raw", self._thermal, 5)
        self.create_subscription(Image, "/camera/image_raw", self._rgb, 5)
        self.create_subscription(
            CameraInfo, "/thermal_camera/camera_info",
            lambda m: setattr(self, "thermal_info", m), 5)
        self.create_subscription(
            CameraInfo, "/camera/camera_info",
            lambda m: setattr(self, "rgb_info", m), 5)

    def _thermal(self, message: Image) -> None:
        self.thermal = np.frombuffer(message.data, np.uint16).reshape(
            message.height, message.width)

    def _rgb(self, message: Image) -> None:
        self.rgb = np.frombuffer(message.data, np.uint8).reshape(
            message.height, message.width, 3)

    def fresh_pair(self, timeout=5.0):
        self.thermal = self.rgb = None
        deadline = time.time() + timeout
        while (self.thermal is None or self.rgb is None) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.thermal, self.rgb


def find_grid(image: np.ndarray, min_area: float):
    detector = blob_detector(min_area)
    found, centres = cv2.findCirclesGrid(
        image, GRID, flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=detector)
    return centres if found else None


def same_orientation(a: np.ndarray, b: np.ndarray) -> bool:
    """Do two detections of a symmetric grid run the same way round?

    findCirclesGrid on a symmetric grid is ambiguous under a 180 degree
    rotation, so one camera can return the points in reverse order. The two
    cameras here are 68 mm apart and see almost the same view, so a genuine
    disagreement means the ordering flipped - and pairing point i with point i
    would then match opposite corners of the board.
    """
    first = (a[-1] - a[0]).ravel()
    second = (b[-1] - b[0]).ravel()
    return float(np.dot(first, second)) > 0


def to_gray(thermal: np.ndarray) -> np.ndarray:
    celsius = thermal / 100.0 - 273.15
    span = max(celsius.max() - celsius.min(), 1e-6)
    return (255 - (celsius - celsius.min()) / span * 255).astype(np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--poses", type=int, default=20)
    parser.add_argument("--settle", type=float, default=1.2)
    arguments = parser.parse_args()

    rclpy.init()
    node = Capture()
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.thermal_info and node.rgb_info:
            break
    if not (node.thermal_info and node.rgb_info):
        missing = []
        if node.rgb_info is None:
            missing.append("/camera/camera_info (시뮬레이터가 떠 있는지 확인)")
        if node.thermal_info is None:
            missing.append(
                "/thermal_camera/camera_info -> "
                "ros2 run hazard_guard_simulation thermal_camera_info.py "
                "또는 camera_view.launch.py 실행"
            )
        print("camera_info 를 받지 못했습니다:")
        for item in missing:
            print(f"  - {item}")
        return 1

    # Distance is the main lever, tilt a small one. The depth axis is only
    # observable through parallax, so views have to differ in range - hence the
    # wide x spread. Tilt helps too, but only a little: a tilted circle
    # projects to an ellipse whose centroid is not the projection of the
    # circle's centre, and with 35 mm circles that bias grows fast. Measured
    # here, +-17 deg of tilt moved the baseline estimate 10 mm the wrong way
    # while +-9 deg did not.
    rng = np.random.default_rng(7)
    poses = []
    for index in range(arguments.poses):
        poses.append((
            0.10 + rng.uniform(-0.40, 0.40),
            -1.4121 + rng.uniform(-0.10, 0.10),
            0.26 + rng.uniform(-0.04, 0.06),
            rng.uniform(-0.15, 0.15),
            rng.uniform(-0.15, 0.15),
            np.pi + rng.uniform(-0.25, 0.25),
        ))

    thermal_points, rgb_points, world_points = [], [], []
    flipped = 0
    board = object_points()
    for index, (x, y, z, roll, pitch, yaw) in enumerate(poses):
        set_target_pose(x, y, z, roll, pitch, yaw)
        time.sleep(arguments.settle)
        thermal, rgb = node.fresh_pair()
        if thermal is None or rgb is None:
            continue
        thermal_centres = find_grid(to_gray(thermal), 12)
        rgb_centres = find_grid(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), 60)
        ok = thermal_centres is not None and rgb_centres is not None
        print(f"  pose {index + 1:2d}/{len(poses)}  "
              f"thermal={'O' if thermal_centres is not None else 'X'} "
              f"rgb={'O' if rgb_centres is not None else 'X'}")
        if ok and not same_orientation(rgb_centres, thermal_centres):
            flipped += 1
            thermal_centres = thermal_centres[::-1].copy()
        if ok:
            thermal_points.append(thermal_centres)
            rgb_points.append(rgb_centres)
            world_points.append(board)

    node.destroy_node()
    rclpy.shutdown()

    print(f"\n사용 가능한 쌍 {len(world_points)} / {len(poses)}"
          f"  (순서 뒤집힘 보정 {flipped}건)")
    if len(world_points) < 6:
        print("쌍이 부족합니다 (최소 6). 판 위치 범위를 좁히거나 poses 를 늘리세요")
        return 1

    thermal_size = (node.thermal_info.width, node.thermal_info.height)
    rgb_size = (node.rgb_info.width, node.rgb_info.height)
    thermal_k = np.array(node.thermal_info.k, np.float64).reshape(3, 3)
    rgb_k = np.array(node.rgb_info.k, np.float64).reshape(3, 3)

    # Intrinsics stay fixed: the simulator's are exact, and on hardware they
    # come from a separate single-camera run. Solving for them here as well
    # would let intrinsic error hide inside the extrinsic.
    error, _, _, _, _, rotation, translation, _, _ = cv2.stereoCalibrate(
        world_points, rgb_points, thermal_points,
        rgb_k, None, thermal_k, None,
        rgb_size, flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7),
    )

    angles = np.degrees(cv2.Rodrigues(rotation)[0]).ravel()
    print(f"\n재투영 오차 {error:.3f} px")
    print(f"이동  x {translation[0][0] * 1000:+.1f}  "
          f"y {translation[1][0] * 1000:+.1f}  z {translation[2][0] * 1000:+.1f} mm")
    print(f"회전  {angles[0]:+.2f}  {angles[1]:+.2f}  {angles[2]:+.2f} deg")

    # Ground truth from the URDF: the thermal camera sits 68 mm to the left of
    # the RGB/depth optical frame, with no rotation. In the optical convention
    # that is +68 mm along x.
    truth = np.array([0.068, 0.0, 0.0])
    offset = (translation.ravel() - truth) * 1000
    print(f"\n정답 대비 이동 오차  {offset[0]:+.1f} {offset[1]:+.1f} {offset[2]:+.1f} mm"
          f"  (크기 {np.linalg.norm(offset):.1f} mm)")
    print(f"정답 대비 회전 오차  {np.linalg.norm(angles):.2f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
