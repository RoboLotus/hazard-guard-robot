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


# How long to let one ign state query run. It serialises the whole world, so
# it is slow by nature - and much slower with the Gazebo GUI attached, where it
# was measured at 37 s against a few seconds headless. A 30 s ceiling timed out
# on every attempt and the run died after six of them having printed nothing,
# which looked like a hang rather than a timeout.
GZ_TIMEOUT = 90.0


def gz(command, attempts=4):
    """Run an ign CLI query, which is slow to answer and sometimes not at all.

    It gets slower again once the board is in the world - 96 tile visuals are a
    lot of state to serialise - so callers ask before spawning where they can.
    """
    for attempt in range(attempts):
        if attempt:
            print(f"    gz 질의 재시도 {attempt + 1}/{attempts} ...", flush=True)
        try:
            done = subprocess.run(command, capture_output=True, text=True,
                                  timeout=GZ_TIMEOUT)
            if done.stdout.strip():
                return done.stdout
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2.0)
    raise RuntimeError(
        f"gz 질의 실패: {' '.join(command)}\n"
        f"  시뮬레이터가 떠 있는지, Gazebo 서버가 하나뿐인지 확인하세요.\n"
        f"  GUI(gui:=true)를 켜면 이 질의가 크게 느려집니다.")


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


# Thermal field of view as half-extent per metre of range: tan(28.5 deg) across
# and tan(22.15 deg) down, for the 160 x 120 / 57 degree sensor.
FIELD_HALF_WIDTH = 0.543
FIELD_HALF_HEIGHT = 0.407
# Thermal optical frame height above the floor, from the URDF.
CAMERA_HEIGHT = 0.134
FLOOR_CLEARANCE = 0.015
# Fraction of the available room an offset view is allowed to use, leaving the
# rest to absorb the board's own rotation.
OFFSET_MARGIN = 0.75

# The thermal camera sits this far to the left of the depth camera the board is
# aimed by, so the two sides do not cost the same. Pushing the board left moves
# it toward the thermal axis and pushing it right moves it away, and a budget
# computed symmetrically overruns on the right by twice this. That is what put
# every screen-right view off the edge in the first near-range run.
THERMAL_BASELINE_LEFT = 0.068


def pose_plan(near: float, mid: float, far: float, spec: dict):
    """(label, distance, lateral, height, roll, pitch, yaw) per view.

    Offsets are computed from the board and the field rather than written in.
    How far the board can be pushed off centre before it leaves frame depends
    on range, so a fixed number is either wasteful at the far end or off-screen
    at the near end - and a view that leaves frame is not a hard view, it is a
    missing one.

    Angles are the board's, not the robot's. Yaw turns it about the vertical,
    pitch tips its top toward or away from the camera, roll spins it in its own
    plane. All three are kept modest: a chequerboard seen far off-normal loses
    corner contrast in the thermal image long before it does in RGB.

    Every category the calibration needs appears at all three ranges. Distance
    is what separates translation from rotation - a sideways shift moves near
    corners more than far ones while a yaw moves both alike - so a category
    seen at one range only cannot help that separation.
    """
    half_width = spec["width_m"] / 2.0
    half_height = spec["height_m"] / 2.0
    plan = []
    for distance, tag in ((near, "near"), (mid, "mid"), (far, "far")):
        room = max(0.0, FIELD_HALF_WIDTH * distance - half_width) * OFFSET_MARGIN
        left = room + THERMAL_BASELINE_LEFT
        right = max(0.0, room - THERMAL_BASELINE_LEFT)
        vertical = max(0.0, FIELD_HALF_HEIGHT * distance - half_height) * OFFSET_MARGIN
        centre_z = max(CAMERA_HEIGHT, half_height + FLOOR_CLEARANCE)
        top_z = centre_z + vertical
        bottom_z = max(half_height + FLOOR_CLEARANCE, centre_z - vertical)
        plan += [
            (f"frontal-{tag}", distance, 0.0, centre_z, 0.0, 0.0, 0.0),
            (f"yaw-left-{tag}", distance, 0.0, centre_z, 0.0, 0.0, +0.32),
            (f"yaw-right-{tag}", distance, 0.0, centre_z, 0.0, 0.0, -0.32),
            (f"pitch-up-{tag}", distance, 0.0, centre_z, 0.0, +0.28, 0.0),
            (f"pitch-down-{tag}", distance, 0.0, centre_z, 0.0, -0.28, 0.0),
            (f"screen-left-{tag}", distance, +left, centre_z, 0.0, 0.0, 0.0),
            (f"screen-right-{tag}", distance, -right, centre_z, 0.0, 0.0, 0.0),
            (f"screen-top-{tag}", distance, 0.0, top_z, 0.0, 0.0, 0.0),
            (f"screen-bottom-{tag}", distance, 0.0, bottom_z, 0.0, 0.0, 0.0),
            # Roll is the one a fronto-parallel board never constrains on its
            # own, and it is also the axis the sensitivity report shows as
            # weakest, so it gets its own view at every range.
            (f"roll-{tag}", distance, 0.0, centre_z, +0.35, 0.0, 0.0),
            # One view per range with everything moving at once. Views that
            # vary a single parameter leave its trade-off partners free; a
            # corner reached from several directions pins both.
            (f"combined-{tag}", distance, +left * 0.6, top_z * 0.9,
             -0.25, +0.18, -0.22),
        ]
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
    parser.add_argument("--near", type=float, default=0.50)
    parser.add_argument("--mid", type=float, default=0.65)
    parser.add_argument("--far", type=float, default=0.80)
    parser.add_argument("--settle", type=float, default=2.5)
    parser.add_argument("--retries", type=int, default=6)
    parser.add_argument("--save-frames", action="store_true",
                        help="원본 이미지도 저장 (해상도 스윕·시각화용, 용량 큼)")
    arguments = parser.parse_args()

    sys.path.insert(0, str(ROOT / "tools"))
    from paper_calib import detect as detector

    spec = json.loads((MODEL_DIR / "target.json").read_text())
    print(f"판  {spec['width_m'] * 1000:.0f} x {spec['height_m'] * 1000:.0f} mm, "
          f"RGB {spec['fine_columns']}x{spec['fine_rows']} 칸, "
          f"대응점 {spec['matched_points']}개/뷰", flush=True)

    # Before the board goes in: the query slows down sharply once it is there,
    # and the robot does not move for the rest of the run anyway. It is also
    # the slowest step, so it says so - a silent minute reads as a hang.
    print("로봇 위치 질의 중 (수십 초 걸립니다) ...", flush=True)
    base_x, base_y, base_yaw = robot_pose()
    print("판 스폰 중 ...", flush=True)
    spawn_target()

    import rclpy
    rclpy.init()
    node = build_node()()
    print("camera_info 대기 중 ...", flush=True)
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

    plan = pose_plan(arguments.near, arguments.mid, arguments.far, spec)
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
        # Kept so a later comparison can quote the acceptance rate without the
        # capture log, which is where the failures would otherwise only live.
        "poses_attempted": np.array(len(plan)),
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
