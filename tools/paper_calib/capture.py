"""Drive the board through a spread of poses and keep the views that detect.

A single board pose cannot separate the six extrinsic numbers: at one distance
a sideways shift and a small yaw move every corner the same way, so the two
trade off freely and the solver picks any point along the trade. Breaking that
needs the board seen near and far (which scales translation's effect but not
rotation's) and seen in every part of the frame (which is what makes a rotation
about the optical axis distinguishable from nothing at all). The pose list
below is built from that requirement rather than from a sweep of convenient
numbers, and each entry is labelled with the category it covers so a gap in the
coverage shows up in the summary instead of hiding in an average.

Views are retried, and a view is only offered to the detector once the thermal
frame actually contains the board. The board's hot tiles are the only thing in
this world at 45 C, so counting pixels near that value is a direct check that
the frame being paired is the right one, for the price of one comparison.

That guard was added while chasing detections that flapped between success and
failure at a fixed pose. The cause turned out to be two Gazebo servers running
against the same world - camera images came from one, board moves went to the
other - and with a single server the same pose now detects six times out of
six. The check is kept anyway: it costs nothing and it fails loudly rather than
quietly feeding a mismatched pair into a calibration, but it is insurance and
not a workaround for anything currently known to be broken.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent
MODEL_DIR = ROOT / "src" / "hazard_guard_simulation" / "models" / "paper_calibration_target"
RESULTS = ROOT / "runtime" / "calibration"

WORLD = "demo_facility_scaled"
TARGET = "paper_cal_target"
ROBOT = "hazard_guard_m1"

# The board's hot tiles. Nothing else in the demo world sits at this
# temperature, so a thermal frame containing none of it is not showing the
# board, whatever the reason.
HOT_C = 45.0
HOT_TOLERANCE_C = 1.0
MIN_HOT_PIXELS = 200


def gz(command, attempts=6):
    """Run an ign CLI query, which is slow to answer and sometimes not at all.

    It gets markedly slower once the board is in the world - 96 tile visuals
    are a lot of state to serialise - so callers ask before spawning where they
    can, and the retry budget is generous where they cannot.
    """
    for _ in range(attempts):
        try:
            done = subprocess.run(command, capture_output=True, text=True, timeout=30)
            if done.stdout.strip():
                return done.stdout
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2.0)
    raise RuntimeError(f"gz 질의 실패: {' '.join(command)}  (시뮬레이터 확인)")


def robot_pose():
    output = gz(["ign", "model", "-m", ROBOT, "-p"])
    triples = [[float(v) for v in match.split()]
               for match in re.findall(r"\[([-0-9.e+ ]+)\]", output)
               if len(match.split()) == 3]
    if len(triples) < 2:
        raise RuntimeError(f"{ROBOT} 위치를 읽지 못했습니다:\n{output}")
    (x, y, _z), (_roll, _pitch, yaw) = triples[0], triples[1]
    return x, y, yaw


def quaternion(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def set_pose(x, y, z, roll, pitch, yaw):
    qx, qy, qz, qw = quaternion(roll, pitch, yaw)
    subprocess.run(
        ["ign", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
         "--timeout", "3000", "--req",
         f'name: "{TARGET}", position: {{x: {x}, y: {y}, z: {z}}}, '
         f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"],
        capture_output=True, check=False)


def spawn_target():
    if TARGET in gz(["ign", "model", "--list"]):
        return
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", TARGET,
         "-file", str(MODEL_DIR / "model.sdf"), "-x", "0", "-y", "0", "-z", "-5"],
        capture_output=True, check=False, timeout=40)
    # 96 tiles take noticeably longer to register than the circle target's 16.
    time.sleep(4.0)


def pose_plan(near: float, mid: float, far: float):
    """(label, distance, lateral, height, roll, pitch, yaw) per view.

    Height is the board centre above the floor; the thermal optical frame sits
    at 0.134 m, and the board is 0.40 m tall, so anything under about 0.21 m
    puts its lower edge through the floor.

    Angles are the board's, not the robot's. Yaw turns it about the vertical,
    pitch tips its top toward or away from the camera; both are kept modest
    because a chequerboard seen far off-normal loses corner contrast in the
    thermal image long before it does in RGB.
    """
    plan = []
    for distance, tag in ((near, "near"), (mid, "mid"), (far, "far")):
        plan.append((f"frontal-{tag}", distance, 0.0, 0.30, 0.0, 0.0, 0.0))
    for distance in (near, mid, far):
        plan.append(("yaw-left", distance, 0.0, 0.30, 0.0, 0.0, +0.30))
        plan.append(("yaw-right", distance, 0.0, 0.30, 0.0, 0.0, -0.30))
    for distance in (near, mid, far):
        plan.append(("tilt-up", distance, 0.0, 0.30, 0.0, +0.25, 0.0))
        plan.append(("tilt-down", distance, 0.0, 0.30, 0.0, -0.25, 0.0))
    for distance in (mid, far):
        plan.append(("screen-left", distance, +0.22, 0.30, 0.0, 0.0, 0.0))
        plan.append(("screen-right", distance, -0.22, 0.30, 0.0, 0.0, 0.0))
        plan.append(("screen-top", distance, 0.0, 0.44, 0.0, 0.0, 0.0))
        plan.append(("screen-bottom", distance, 0.0, 0.23, 0.0, 0.0, 0.0))
    # Rotation about the optical axis is the one a fronto-parallel board never
    # constrains, so a few views carry it explicitly.
    for distance in (mid, far):
        plan.append(("roll", distance, 0.0, 0.30, +0.30, 0.0, 0.0))
        plan.append(("combined", distance, +0.15, 0.36, -0.20, +0.15, -0.20))
    return plan


class Capture:
    """Synchronised RGB/thermal grabs plus the camera matrices."""

    def __init__(self, node_class):
        self.node = node_class()

    def close(self):
        self.node.destroy_node()


def build_node():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    class Grab(Node):
        def __init__(self):
            super().__init__("paper_calibration_capture")
            self.rgb = self.thermal = None
            self.rgb_info = self.thermal_info = None
            self.create_subscription(Image, "/camera/image_raw", self._rgb, 5)
            self.create_subscription(Image, "/thermal_camera/image_raw", self._thermal, 5)
            self.create_subscription(CameraInfo, "/camera/camera_info",
                                     lambda m: setattr(self, "rgb_info", m), 5)
            self.create_subscription(CameraInfo, "/thermal_camera/camera_info",
                                     lambda m: setattr(self, "thermal_info", m), 5)

        def _rgb(self, message):
            self.rgb = np.frombuffer(message.data, np.uint8).reshape(
                message.height, message.width, 3)

        def _thermal(self, message):
            self.thermal = np.frombuffer(message.data, np.uint16).reshape(
                message.height, message.width)

        def fresh(self, timeout=6.0):
            self.rgb = self.thermal = None
            deadline = time.time() + timeout
            while (self.rgb is None or self.thermal is None) and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
            return self.rgb, self.thermal

        def wait_for_info(self, timeout=25.0):
            deadline = time.time() + timeout
            while (self.rgb_info is None or self.thermal_info is None) \
                    and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
            return self.rgb_info is not None and self.thermal_info is not None

    return Grab


def board_is_rendered(thermal: np.ndarray) -> int:
    celsius = thermal.astype(np.float32) / 100.0 - 273.15
    return int(np.count_nonzero(np.abs(celsius - HOT_C) < HOT_TOLERANCE_C))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(RESULTS / "paper_views.npz"))
    parser.add_argument("--near", type=float, default=1.1)
    parser.add_argument("--mid", type=float, default=1.4)
    parser.add_argument("--far", type=float, default=1.7)
    parser.add_argument("--settle", type=float, default=2.5)
    parser.add_argument("--retries", type=int, default=6)
    parser.add_argument("--save-frames", action="store_true",
                        help="원본 이미지도 저장 (해상도 스윕·시각화용, 용량 큼)")
    arguments = parser.parse_args()

    sys.path.insert(0, str(ROOT / "tools"))
    from paper_calib import detect as detector

    spec = json.loads((MODEL_DIR / "target.json").read_text())
    # Before the board goes in: the query slows down sharply once it is there,
    # and the robot does not move for the rest of the run anyway.
    base_x, base_y, base_yaw = robot_pose()
    spawn_target()

    import rclpy
    rclpy.init()
    node = build_node()()
    if not node.wait_for_info():
        print("camera_info 를 받지 못했습니다:")
        if node.rgb_info is None:
            print("  - /camera/camera_info (시뮬레이터 확인)")
        if node.thermal_info is None:
            print("  - /thermal_camera/camera_info -> "
                  "ros2 run hazard_guard_simulation thermal_camera_info.py")
        rclpy.shutdown()
        return 1

    print(f"로봇 x {base_x:+.2f} y {base_y:+.2f} yaw {np.degrees(base_yaw):+.0f} deg")
    forward = np.array([np.cos(base_yaw), np.sin(base_yaw)])
    sideways = np.array([-np.sin(base_yaw), np.cos(base_yaw)])

    plan = pose_plan(arguments.near, arguments.mid, arguments.far)
    views, frames, ladder, by_label = [], [], {}, {}

    for index, (label, distance, lateral, height, roll, pitch, yaw) in enumerate(plan):
        centre = np.array([base_x, base_y]) + forward * distance + sideways * lateral
        set_pose(centre[0], centre[1], height, roll, pitch, base_yaw + np.pi + yaw)
        time.sleep(arguments.settle)

        accepted = None
        for attempt in range(arguments.retries):
            rgb, thermal = node.fresh()
            if rgb is None or thermal is None:
                continue
            if board_is_rendered(thermal) < MIN_HOT_PIXELS:
                # Thermal scene has not caught up with the move yet.
                time.sleep(0.5)
                continue
            pair, info = detector.detect_pair(rgb, thermal, spec)
            if pair is not None:
                accepted = (pair, info, rgb, thermal)
                break
            time.sleep(0.4)

        status = "X"
        if accepted is not None:
            pair, info, rgb, thermal = accepted
            views.append({
                "label": label,
                "rgb_matched": pair["rgb_matched"],
                "tir_matched": pair["tir_matched"],
                "rgb_corners": pair["rgb_corners"],
                "object_matched": pair["object_matched"],
                "object_points": pair["object_points"],
                "tir_how": info["tir_how"],
            })
            if arguments.save_frames:
                frames.append((rgb.copy(), thermal.copy()))
            ladder[info["tir_how"]] = ladder.get(info["tir_how"], 0) + 1
            status = "O"
        by_label.setdefault(label, [0, 0])
        by_label[label][status == "O"] += 1
        print(f"  {index + 1:2d}/{len(plan)} {label:14s} d={distance:.1f} {status}")

    node.destroy_node()
    rclpy.shutdown()

    print(f"\n채택 {len(views)} / {len(plan)}")
    print("범주별 (실패/성공):")
    for label, (failed, ok) in sorted(by_label.items()):
        print(f"  {label:14s} {ok}/{ok + failed}")
    print(f"열화상 검출 단계: {ladder}")

    if len(views) < 6:
        print("뷰가 부족합니다. 거리 범위나 판 높이를 조정하세요")
        return 1

    out = Path(arguments.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "labels": np.array([v["label"] for v in views]),
        "tir_how": np.array([v["tir_how"] for v in views]),
        "rgb_matched": np.stack([v["rgb_matched"] for v in views]),
        "tir_matched": np.stack([v["tir_matched"] for v in views]),
        "rgb_corners": np.stack([v["rgb_corners"] for v in views]),
        "object_matched": np.stack([v["object_matched"] for v in views]),
        "object_points": views[0]["object_points"],
        "rgb_k": np.array(node.rgb_info.k, np.float64),
        "thermal_k": np.array(node.thermal_info.k, np.float64),
        "rgb_size": np.array([node.rgb_info.width, node.rgb_info.height]),
        "thermal_size": np.array([node.thermal_info.width, node.thermal_info.height]),
    }
    if frames:
        payload["rgb_frames"] = np.stack([f[0] for f in frames])
        payload["thermal_frames"] = np.stack([f[1] for f in frames])
    np.savez_compressed(out, **payload)
    print(f"저장  {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
