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
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

WORLD = "demo_facility_scaled"
TARGET = "cal_target"
ROBOT = "hazard_guard_m1"
PACKAGE = Path(__file__).resolve().parent.parent / "src" / "hazard_guard_simulation"
MODEL_DIR = PACKAGE / "models" / "calibration_target"
RESULTS = Path(__file__).resolve().parent.parent / "runtime" / "calibration"

SPEC = json.loads((MODEL_DIR / "target.json").read_text())
GRID = (SPEC["columns"], SPEC["rows"])
SPACING = SPEC["spacing_m"]

# Thermal optical frame height above the floor, from the URDF.
CAMERA_HEIGHT = 0.134


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


def gz(command, attempts=3):
    """Run an ign CLI query. It is slow to answer and sometimes not at all."""
    for attempt in range(attempts):
        try:
            done = subprocess.run(
                command, capture_output=True, text=True, timeout=30
            )
            if done.stdout.strip():
                return done.stdout
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    raise RuntimeError(
        f"gz 질의 실패: {' '.join(command)}  (시뮬레이터가 떠 있는지 확인)"
    )


def robot_pose():
    """(x, y, yaw) of the robot in world coordinates, straight from Gazebo.

    The target has to be placed in front of wherever the robot actually is.
    Hard-coding a world position only works while the robot sits at one spawn
    point, and puts the board behind it otherwise.
    """
    output = gz(["ign", "model", "-m", ROBOT, "-p"])
    # "ign model -p" prints the pose as two bracketed triples, XYZ then RPY,
    # among other bracketed text that is not numeric.
    triples = [
        [float(v) for v in match.split()]
        for match in re.findall(r"\[([-0-9.e+ ]+)\]", output)
        if len(match.split()) == 3
    ]
    if len(triples) < 2:
        raise RuntimeError(f"{ROBOT} 의 위치를 읽지 못했습니다:\n{output}")
    (x, y, _z), (_roll, _pitch, yaw) = triples[0], triples[1]
    return x, y, yaw


def spawn_target() -> None:
    """Put the board in the world if it is not there yet."""
    listed = gz(["ign", "model", "--list"])
    if TARGET in listed:
        return
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", WORLD, "-name", TARGET,
         "-file", str(MODEL_DIR / "model.sdf"),
         "-x", "0", "-y", "0", "-z", "-5"],   # parked underground until posed
        capture_output=True, check=False, timeout=30,
    )
    time.sleep(2.0)


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
    parser.add_argument("--near", type=float, default=0.50)
    parser.add_argument("--far", type=float, default=0.75)
    arguments = parser.parse_args()

    spawn_target()
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

    # Poses are built around wherever the robot is, in its own forward
    # direction, so the board always lands in front of the camera.
    #
    # Range is the main lever: the depth axis is only observable through
    # parallax, which goes as 1/Z. The window is narrow at both ends - closer
    # than 0.5 m and the 320 mm board leaves the 44 deg vertical field, further
    # than 0.75 m and a 40 mm circle drops under 8 px - so the spread around it
    # is kept tight enough that jitter does not push the board out of frame.
    # Tilt helps a little but has to stay small -
    # a tilted circle projects to an ellipse whose centroid is not the
    # projection of the circle's centre, and past about +-9 deg that bias costs
    # more than the extra parallax buys.
    base_x, base_y, base_yaw = robot_pose()
    print(f"로봇 위치 x {base_x:+.2f}  y {base_y:+.2f}  yaw {np.degrees(base_yaw):+.0f} deg")
    forward = np.array([np.cos(base_yaw), np.sin(base_yaw)])
    sideways = np.array([-np.sin(base_yaw), np.cos(base_yaw)])

    rng = np.random.default_rng(7)
    poses = []
    for index in range(arguments.poses):
        distance = rng.uniform(arguments.near, arguments.far)
        offset = rng.uniform(-0.035, 0.035)
        centre = np.array([base_x, base_y]) + forward * distance + sideways * offset
        poses.append((
            centre[0], centre[1],
            CAMERA_HEIGHT + rng.uniform(-0.02, 0.03),
            rng.uniform(-0.15, 0.15),
            rng.uniform(-0.15, 0.15),
            base_yaw + np.pi + rng.uniform(-0.20, 0.20),
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

    # Saved so runs can be compared: every change to the board or the pose
    # spread is an experiment, and without a record they are just impressions.
    RESULTS.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    result = {
        "target": SPEC,
        "poses_requested": len(poses),
        "pairs_used": len(world_points),
        "flipped_corrected": flipped,
        "range_m": [arguments.near, arguments.far],
        "reprojection_error_px": round(float(error), 4),
        "translation_mm": [round(float(v) * 1000, 2) for v in translation.ravel()],
        "rotation_deg": [round(float(v), 3) for v in angles],
        "truth_translation_mm": [round(float(v) * 1000, 2) for v in truth],
        "translation_error_mm": [round(float(v), 2) for v in offset],
        "translation_error_norm_mm": round(float(np.linalg.norm(offset)), 2),
        "rotation_error_deg": round(float(np.linalg.norm(angles)), 3),
    }
    path = RESULTS / f"thermal_rgb_{stamp}.json"
    path.write_text(json.dumps(result, indent=2) + "\n")
    (RESULTS / "latest.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"\n저장  {path.relative_to(Path(__file__).resolve().parent.parent)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
