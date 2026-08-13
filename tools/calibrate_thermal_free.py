"""Calibrate the thermal camera against the depth camera without a target.

The circle-board run in calibrate_thermal_rgb.py needs a board, a person to
hold it, and a procedure. This one needs none of them: it aligns the geometry
the scene already provides.

The idea is the one Levinson and Thrun used for lasers and cameras. A depth
discontinuity is an object boundary, and an object boundary is almost always a
temperature step too - a different object, a different emissivity, a different
temperature. So project the depth camera's discontinuity pixels into the
thermal image through a candidate extrinsic and add up how much thermal edge
they land on. The right extrinsic is the one that piles them onto the edges.

What makes this workable here is that the robot carries a depth camera. Two
plain 2D cameras cannot be aligned this way - without depth there is no way to
warp one view into the other. With depth every pixel is a 3D point, and the
problem becomes the same shape as laser-to-camera calibration.

The forward model is already in thermal_overlay.py: depth pixel -> 3D point ->
extrinsic -> thermal pixel. This file wraps a score around it and searches.

Two subcommands, deliberately split. The objective gets tuned many times - blur
width, edge threshold, how many points - and there is no reason to drive the
robot again for each attempt. `record` collects frames once, `solve` runs
offline on them and needs no ROS at all.

    python3 tools/calibrate_thermal_free.py record --out runtime/calibration/frames.npz
    python3 tools/calibrate_thermal_free.py solve --frames runtime/calibration/frames.npz
    python3 tools/calibrate_thermal_free.py selftest
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "runtime" / "calibration"

# Where the mounting drawing says the thermal camera is: 68 mm to the left of
# the depth optical frame, no rotation. This is the search's starting point on
# any robot, simulated or not.
CAD_MM = np.array([68.0, 0.0, 0.0])

# What the simulator actually renders, which is not the same thing.
#
# None of the <sensor> blocks in the URDF carry a <pose>, so Fortress puts each
# camera at its link origin and the optical-frame joints - 30/0/12 mm for
# depth, 9/0/0 mm for thermal - never reach the renderer. gz_frame_id only
# labels the published frame. The two link origins are 21/68/12 mm apart in
# base_link, which in the optical convention is:
#
#     T = (+68, +12, -21) mm
#
# The board run reported (66.18, 12.22, -20.39). Those three numbers were read
# as a systematic error and blamed on centroid bias in the thermal blobs; they
# are the URDF-to-Gazebo discrepancy, and the board method was accurate to
# about 2 mm all along. Scoring against the drawing instead of against what was
# rendered would charge this method for the same thing.
TRUTH_MM = np.array([68.0, 12.0, -21.0])

# Only depth pixels whose neighbourhood jumps by at least this much count as a
# boundary. Below it the "edge" is depth noise on a flat surface.
EDGE_MIN_M = 0.05
# ponytail: a single large jump would otherwise dominate the sum; clipping the
# weight is the cheapest way to keep one doorway from outvoting a whole frame.
EDGE_CLIP_M = 0.50
# Points kept per frame, strongest edges first. More is slower, not better -
# the weakest edges are the noisiest.
POINTS_PER_FRAME = 1500

# Coarse to fine, in thermal pixels. A wider blur widens the basin the search
# can fall into; a narrow one is what actually resolves the answer.
#
# The first attempt started at 9 px and that destroyed the run: on a 160 px
# frame a 9 px blur smears the edge map into one broad hill whose summit has
# nothing to do with the geometry, and the stage walked 800 mm away. The later
# stages are local searches and cannot come back. Nothing wider than about 4 px
# survives on a frame this small, and it costs nothing: the mounting drawing is
# already good to a few millimetres, which is a couple of pixels at working
# range.
BLUR_SIGMAS = (4.0, 2.5, 1.5)

# How far from the mounting drawing the search may wander. A camera bolted to a
# bracket is not 5 cm from where the drawing puts it, and letting the optimiser
# explore that far only offers it degenerate ways to score well.
SEARCH_MM = 40.0
SEARCH_DEG = 5.0


# --------------------------------------------------------------------------
# geometry and scoring - no ROS here, this half runs anywhere
# --------------------------------------------------------------------------

def depth_edges(depth: np.ndarray) -> np.ndarray:
    """How much farther away the farthest 4-neighbour is, in metres.

    Signed, not absolute, and that is the important part. A depth
    discontinuity has two sides, but only the near one is an occluding
    contour, and only an occluding contour is what the thermal camera sees as
    an edge. Pixels on the far side belong to a surface that continues behind
    the near object; viewed from 68 mm away they land f*b*(1/Z_near - 1/Z_far)
    off the boundary - about 14 px here - so scoring them against the thermal
    edge drags the answer sideways along the baseline. Keeping only pixels
    that have a farther neighbour keeps only the near side.

    Real depth cameras also return 0 or NaN in whole patches - reflections,
    dark surfaces, anything inside the near clip. Those holes have huge
    "jumps" at their border that mean nothing, so invalid pixels take part in
    no difference at all.
    """
    valid = np.isfinite(depth) & (depth > 0)
    filled = np.where(valid, depth, np.nan)
    jumps = [
        np.roll(filled, shift, axis=axis) - filled
        for shift, axis in ((1, 0), (-1, 0), (1, 1), (-1, 1))
    ]
    # fmax rather than nanmax: a pixel whose neighbours are all invalid gives
    # an all-NaN slice, which nanmax answers correctly but complains about on
    # every frame.
    edge = np.fmax.reduce(np.stack(jumps), axis=0)
    edge = np.nan_to_num(edge, nan=0.0, posinf=0.0)
    edge[~valid] = 0.0
    # Rolling wraps around; the border differences it invents are not edges.
    edge[0, :] = edge[-1, :] = edge[:, 0] = edge[:, -1] = 0.0
    return edge.astype(np.float32)


def thermal_edges(thermal: np.ndarray) -> np.ndarray:
    """Gradient magnitude of the temperature image.

    The stream is Kelvin x 100 in uint16, so it is converted to celsius first
    - not because the optimiser cares about the offset, but so the threshold
    and the numbers printed alongside it are in the unit everything else here
    uses.
    """
    celsius = thermal.astype(np.float32) / 100.0 - 273.15
    dx = cv2.Sobel(celsius, cv2.CV_32F, 1, 0, ksize=3)
    dy = cv2.Sobel(celsius, cv2.CV_32F, 0, 1, ksize=3)
    return cv2.magnitude(dx, dy)


def edge_points(depth: np.ndarray, k: np.ndarray, limit: int = POINTS_PER_FRAME):
    """Boundary pixels of a depth image as 3D points with weights."""
    edge = depth_edges(depth)
    v, u = np.nonzero(edge >= EDGE_MIN_M)
    if v.size == 0:
        return np.zeros((0, 3), np.float64), np.zeros(0, np.float64)
    weight = np.minimum(edge[v, u], EDGE_CLIP_M)
    if v.size > limit:
        strongest = np.argpartition(weight, -limit)[-limit:]
        v, u, weight = v[strongest], u[strongest], weight[strongest]
    z = depth[v, u].astype(np.float64)
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    points = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
    return points, weight.astype(np.float64)


def transform(params: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Unpack the search vector into a rotation and a translation.

    Translation is carried in millimetres and rotation in degrees so the six
    numbers are the same order of magnitude. Powell searches one coordinate at
    a time and a vector mixing 0.068 with 0.002 makes its step sizes wrong for
    one half or the other.
    """
    rotation = cv2.Rodrigues(np.radians(params[3:6]))[0]
    return rotation, params[0:3] / 1000.0


def project(points: np.ndarray, params: np.ndarray, k: np.ndarray):
    rotation, translation = transform(params)
    moved = points @ rotation.T + translation
    z = moved[:, 2]
    ahead = z > 1e-6
    u = np.full(z.shape, -1.0)
    v = np.full(z.shape, -1.0)
    u[ahead] = k[0, 0] * moved[ahead, 0] / z[ahead] + k[0, 2]
    v[ahead] = k[1, 1] * moved[ahead, 1] / z[ahead] + k[1, 2]
    return u, v, ahead


def sample(image: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Bilinear lookup, zero outside the frame.

    Bilinear rather than nearest because the score has to change smoothly as
    the extrinsic moves. With nearest sampling the objective is a staircase
    and Powell stalls on the flat parts.
    """
    height, width = image.shape
    inside = (u >= 0) & (u <= width - 1.001) & (v >= 0) & (v <= height - 1.001)
    out = np.zeros(u.shape, np.float64)
    if not np.any(inside):
        return out
    x, y = u[inside], v[inside]
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    out[inside] = (
        image[y0, x0] * (1 - fx) * (1 - fy)
        + image[y0, x0 + 1] * fx * (1 - fy)
        + image[y0 + 1, x0] * (1 - fx) * fy
        + image[y0 + 1, x0 + 1] * fx * fy
    )
    return out


def prepare(frames: dict, fx_error_pct: float = 0.0):
    """Turn stored images into the points and edge maps the score needs."""
    depth_k = frames["depth_k"].reshape(3, 3).astype(np.float64)
    thermal_k = frames["thermal_k"].reshape(3, 3).astype(np.float64)
    if fx_error_pct:
        # For the leak experiment: on real hardware the thermal intrinsics come
        # from a datasheet field of view, not a measurement. Deliberately
        # spoiling them here shows how much of that error the extrinsic
        # absorbs.
        thermal_k = thermal_k.copy()
        thermal_k[0, 0] *= 1.0 + fx_error_pct / 100.0
        thermal_k[1, 1] *= 1.0 + fx_error_pct / 100.0
    prepared = []
    for depth, thermal in zip(frames["depth"], frames["thermal"]):
        points, weight = edge_points(depth, depth_k)
        if points.shape[0] < 50:
            continue
        prepared.append((points, weight, thermal_edges(thermal)))
    return prepared, thermal_k


def blur_stage(prepared: list, sigma: float) -> list:
    """Blurring inside the objective would redo the same convolution on every
    evaluation; it only has to happen once per stage."""
    return [(p, w, cv2.GaussianBlur(e, (0, 0), sigma)) for p, w, e in prepared]


def score(params: np.ndarray, prepared: list, thermal_k: np.ndarray, sigma: float) -> float:
    """How much thermal edge the depth boundaries land on. Higher is better."""
    return score_prepared(params, blur_stage(prepared, sigma), thermal_k)


def solve_extrinsic(prepared: list, thermal_k: np.ndarray, start: np.ndarray, quiet=False):
    """Coarse-to-fine search for the six numbers, bounded around the drawing."""
    params = np.asarray(start, np.float64).copy()
    anchor = np.asarray(start, np.float64)
    bounds = [(anchor[i] - SEARCH_MM, anchor[i] + SEARCH_MM) for i in range(3)]
    bounds += [(anchor[i] - SEARCH_DEG, anchor[i] + SEARCH_DEG) for i in range(3, 6)]
    for sigma in BLUR_SIGMAS:
        staged = blur_stage(prepared, sigma)
        result = minimize(
            lambda x: -score_prepared(x, staged, thermal_k),
            params, method="Powell", bounds=bounds,
            options={"xtol": 1e-3, "ftol": 1e-6, "maxiter": 20000},
        )
        params = result.x
        if not quiet:
            print(f"  blur {sigma:4.1f} px  점수 {-result.fun:.4f}  "
                  f"이동 {params[0]:+.1f} {params[1]:+.1f} {params[2]:+.1f} mm  "
                  f"회전 {params[3]:+.2f} {params[4]:+.2f} {params[5]:+.2f} deg")
    return params, -result.fun


def score_prepared(params: np.ndarray, staged: list, thermal_k: np.ndarray) -> float:
    """Same score, but on frames whose edge maps are already blurred."""
    total = 0.0
    weight_sum = 0.0
    for points, weight, blurred in staged:
        u, v, ahead = project(points, params, thermal_k)
        total += float(np.sum(weight * ahead * sample(blurred, u, v)))
        weight_sum += float(np.sum(weight))
    return total / max(weight_sum, 1e-9)


# --------------------------------------------------------------------------
# record - the only half that needs ROS
# --------------------------------------------------------------------------

def record(arguments) -> int:
    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("calibration_free_record")
            self.depth_info = self.thermal_info = None
            self.pose = None          # (x, y, yaw)
            self.moving = True
            self.last_saved_pose = None
            self.depth_frames: list = []
            self.thermal_frames: list = []
            self.rejected = {"moving": 0, "too_far": 0, "flat": 0, "not_new": 0}

            self.create_subscription(
                CameraInfo, arguments.depth_info,
                lambda m: setattr(self, "depth_info", m), 10)
            self.create_subscription(
                CameraInfo, arguments.thermal_info,
                lambda m: setattr(self, "thermal_info", m), 10)
            self.create_subscription(Odometry, arguments.odom, self.on_odom, 10)
            synchronizer = ApproximateTimeSynchronizer(
                [Subscriber(self, Image, arguments.depth_image),
                 Subscriber(self, Image, arguments.thermal_image)],
                queue_size=10, slop=0.15)
            synchronizer.registerCallback(self.on_pair)

        def on_odom(self, message: Odometry) -> None:
            twist = message.twist.twist
            # Thermal runs at 8.7 Hz against depth's 10, so a moving robot
            # pairs frames taken from different places. Waiting for a standstill
            # removes the whole synchronisation problem for the price of only
            # shooting at patrol waypoints.
            self.moving = (abs(twist.linear.x) > 0.02 or abs(twist.angular.z) > 0.05)
            position = message.pose.pose.position
            q = message.pose.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (position.x, position.y, yaw)

        def is_new_viewpoint(self) -> bool:
            """Standing still at a waypoint produces the same picture over and
            over. Only the first frame from each place carries information."""
            if self.pose is None:
                return False
            if self.last_saved_pose is None:
                return True
            dx = self.pose[0] - self.last_saved_pose[0]
            dy = self.pose[1] - self.last_saved_pose[1]
            dyaw = abs(np.arctan2(np.sin(self.pose[2] - self.last_saved_pose[2]),
                                  np.cos(self.pose[2] - self.last_saved_pose[2])))
            return np.hypot(dx, dy) > arguments.min_move or dyaw > np.radians(15.0)

        def on_pair(self, depth_message: Image, thermal_message: Image) -> None:
            if len(self.depth_frames) >= arguments.frames:
                return
            if self.depth_info is None or self.thermal_info is None:
                return
            if self.moving:
                self.rejected["moving"] += 1
                return
            if not self.is_new_viewpoint():
                self.rejected["not_new"] += 1
                return

            depth = np.frombuffer(depth_message.data, np.float32).reshape(
                depth_message.height, depth_message.width)
            thermal = np.frombuffer(thermal_message.data, np.uint16).reshape(
                thermal_message.height, thermal_message.width)

            valid = depth[np.isfinite(depth) & (depth > 0)]
            # A frame of nothing but far wall fixes the rotation and says
            # nothing about the translation: parallax from a 68 mm baseline
            # goes as 1/Z.
            if valid.size == 0 or np.median(valid) > arguments.max_depth:
                self.rejected["too_far"] += 1
                return
            if np.count_nonzero(depth_edges(depth) >= EDGE_MIN_M) < 300:
                self.rejected["flat"] += 1
                return

            self.depth_frames.append(depth.copy())
            self.thermal_frames.append(thermal.copy())
            self.last_saved_pose = self.pose
            # Flushed because this runs for minutes alongside a patrol and its
            # output is usually being watched through a redirect.
            print(f"  프레임 {len(self.depth_frames):3d}/{arguments.frames}  "
                  f"중앙 깊이 {np.median(valid):.2f} m", flush=True)

    rclpy.init()
    node = Collector()
    print("정지 중이고 근거리 구조가 있는 프레임만 모읍니다. 순찰을 돌리세요.")
    deadline = time.time() + arguments.timeout
    while len(node.depth_frames) < arguments.frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    depth_info, thermal_info = node.depth_info, node.thermal_info
    depth_frames, thermal_frames = node.depth_frames, node.thermal_frames
    rejected = node.rejected
    node.destroy_node()
    rclpy.shutdown()

    if depth_info is None or thermal_info is None:
        print("camera_info 를 받지 못했습니다 (시뮬레이터와 thermal_camera_info.py 확인)")
        return 1
    if not depth_frames:
        print(f"프레임을 하나도 모으지 못했습니다. 기각 사유: {rejected}")
        return 1

    out = Path(arguments.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # Appending lets a short patrol be topped up by a second lap instead of
    # having to get everything in one run.
    if out.exists() and arguments.append:
        old = np.load(out)
        depth_frames = list(old["depth"]) + depth_frames
        thermal_frames = list(old["thermal"]) + thermal_frames
    np.savez_compressed(
        out,
        depth=np.stack(depth_frames).astype(np.float32),
        thermal=np.stack(thermal_frames).astype(np.uint16),
        depth_k=np.array(depth_info.k, np.float64),
        thermal_k=np.array(thermal_info.k, np.float64),
    )
    print(f"\n{len(depth_frames)} 프레임 저장  {out}")
    print(f"기각 사유: {rejected}")
    return 0


# --------------------------------------------------------------------------
# solve
# --------------------------------------------------------------------------

def solve(arguments) -> int:
    frames = np.load(arguments.frames)
    prepared, thermal_k = prepare(frames, arguments.fx_error)
    print(f"프레임 {len(prepared)} / {len(frames['depth'])}  "
          f"(경계 픽셀이 부족한 프레임은 제외)")
    if len(prepared) < arguments.min_frames:
        print(f"프레임이 부족합니다 (최소 {arguments.min_frames}). 순찰을 한 바퀴 더 돌리세요")
        return 1

    start = np.array([*TRUTH_MM, 0.0, 0.0, 0.0]) if arguments.start_at_truth else \
        np.array(arguments.start, np.float64)
    print(f"초기값 (CAD 장착값)  이동 {start[0]:+.1f} {start[1]:+.1f} {start[2]:+.1f} mm")

    params, final = solve_extrinsic(prepared, thermal_k, start)

    if arguments.perturb:
        # No ground truth exists on the real robot, so the question there is
        # not "is it right" but "does it land in the same place regardless of
        # where it started". This measures that.
        print(f"\n재현성 검사: 초기값을 흔들어 {arguments.perturb}회")
        rng = np.random.default_rng(11)
        landings = []
        for run in range(arguments.perturb):
            jitter = np.concatenate([rng.uniform(-20, 20, 3), rng.uniform(-3, 3, 3)])
            landed, _ = solve_extrinsic(prepared, thermal_k, start + jitter, quiet=True)
            landings.append(landed)
            print(f"  {run + 1}/{arguments.perturb}  이동 "
                  f"{landed[0]:+.1f} {landed[1]:+.1f} {landed[2]:+.1f} mm")
        spread = np.std(np.array(landings), axis=0)
        print(f"  산포 (1σ)  이동 {spread[0]:.1f} {spread[1]:.1f} {spread[2]:.1f} mm  "
              f"회전 {spread[3]:.2f} {spread[4]:.2f} {spread[5]:.2f} deg")

    offset = params[0:3] - TRUTH_MM
    print(f"\n최종  이동 {params[0]:+.1f} {params[1]:+.1f} {params[2]:+.1f} mm")
    print(f"      회전 {params[3]:+.2f} {params[4]:+.2f} {params[5]:+.2f} deg")
    print(f"정답 대비 이동 오차 {offset[0]:+.1f} {offset[1]:+.1f} {offset[2]:+.1f} mm "
          f"(크기 {np.linalg.norm(offset):.1f} mm)")
    print(f"정답 대비 회전 오차 {np.linalg.norm(params[3:6]):.2f} deg")

    # Reject nonsense rather than publishing it. Measured against the drawing,
    # not against the truth, because on the real robot there is no truth to
    # measure against and this guard has to work there too.
    from_cad = np.linalg.norm(params[0:3] - CAD_MM)
    accepted = from_cad < 30.0 and np.linalg.norm(params[3:6]) < 10.0
    if not accepted:
        print("\n기각: CAD 장착값에서 너무 멉니다. 이 결과는 쓰지 마세요")

    RESULTS.mkdir(parents=True, exist_ok=True)
    result = {
        "method": "target-free depth-thermal edge alignment",
        "frames_used": len(prepared),
        "blur_sigmas_px": list(BLUR_SIGMAS),
        "fx_error_pct": arguments.fx_error,
        "score": round(float(final), 5),
        "accepted": bool(accepted),
        "translation_mm": [round(float(v), 2) for v in params[0:3]],
        "rotation_deg": [round(float(v), 3) for v in params[3:6]],
        "truth_translation_mm": [float(v) for v in TRUTH_MM],
        "translation_error_mm": [round(float(v), 2) for v in offset],
        "translation_error_norm_mm": round(float(np.linalg.norm(offset)), 2),
        "rotation_error_deg": round(float(np.linalg.norm(params[3:6])), 3),
    }
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = RESULTS / f"thermal_free_{stamp}.json"
    path.write_text(json.dumps(result, indent=2) + "\n")
    if accepted:
        (RESULTS / "latest_free.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"\n저장  {path.relative_to(ROOT)}")
    return 0


# --------------------------------------------------------------------------
# selftest - synthetic pair with a known answer, no ROS and no simulator
# --------------------------------------------------------------------------

def synthetic(truth_mm=TRUTH_MM, count=6, seed=3):
    """Build depth/thermal pairs that agree exactly under a known extrinsic.

    Both images are rendered from one list of flat plates - a polygon, a depth,
    a temperature - each camera drawing them back to front on its own. That
    detail is the whole trick. Rendering the thermal frame by re-projecting the
    depth camera's pixels instead looks equivalent and is not: the second
    viewpoint sees round the edge of every near object, and those newly
    revealed bands have no pixel to come from. At a 68 mm baseline the band is
    f*b*(1/Z_near - 1/Z_far) wide - about 14 px of a 160 px frame - so every
    boundary gains a false second edge offset from the true one, and the
    optimum moves off the truth. Drawing each plate independently fills those
    bands with the surface that is really behind.

    A plate at a constant depth projects to a plain affine map of itself, so
    "render it in the other camera" is just moving its corners.

    If the solver cannot recover the offset from data with no noise and no
    modelling error, nothing it says about real frames is worth reading.
    """
    rng = np.random.default_rng(seed)
    depth_k = np.array([[520.0, 0, 320.0], [0, 520.0, 240.0], [0, 0, 1.0]])
    thermal_k = np.array([[147.3, 0, 80.0], [0, 147.3, 60.0], [0, 0, 1.0]])
    rotation, translation = transform(np.array([*truth_mm, 0.0, 0.0, 0.0]))

    # The thermal frame is drawn 4x oversized and averaged down. A real
    # detector integrates over the whole pixel, and drawing straight into a
    # 160 x 120 grid instead quantises every boundary to the nearest pixel -
    # half a thermal pixel is 5 mm of baseline at 1.5 m, which is the size of
    # the answer being measured.
    SUPER = 4

    def to_thermal(corners: np.ndarray, z: float) -> np.ndarray:
        points = np.stack([
            (corners[:, 0] - depth_k[0, 2]) * z / depth_k[0, 0],
            (corners[:, 1] - depth_k[1, 2]) * z / depth_k[1, 1],
            np.full(len(corners), z)], axis=1)
        moved = points @ rotation.T + translation
        u = thermal_k[0, 0] * moved[:, 0] / moved[:, 2] + thermal_k[0, 2]
        v = thermal_k[1, 1] * moved[:, 1] / moved[:, 2] + thermal_k[1, 2]
        return np.stack([(u + 0.5) * SUPER - 0.5, (v + 0.5) * SUPER - 0.5], axis=1)

    depths, thermals = [], []
    for _ in range(count):
        # Back wall first, then plates in front of it, far to near.
        plates = [(np.array([[-2000.0, -2000], [3000, -2000], [3000, 3000],
                             [-2000, 3000]]), 3.5, 20.0)]
        for _ in range(8):
            centre = rng.uniform([70, 70], [570, 410])
            size = rng.uniform([50, 50], [190, 190])
            angle = rng.uniform(0, np.pi)
            # A rotated rectangle: boundaries that are not all horizontal and
            # vertical, so no shift can accidentally line one edge up with
            # another.
            unit = np.array([[-1.0, -1], [1, -1], [1, 1], [-1, 1]]) * size / 2
            spin = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
            plates.append((unit @ spin.T + centre,
                           float(rng.uniform(0.6, 2.5)),
                           float(rng.uniform(25.0, 60.0))))
        plates.sort(key=lambda plate: -plate[1])

        depth = np.zeros((480, 640), np.float32)
        fine = np.full((120 * SUPER, 160 * SUPER), 20.0, np.float32)
        for corners, z, celsius in plates:
            cv2.fillPoly(depth, [np.round(corners).astype(np.int32)], float(z))
            cv2.fillPoly(fine, [np.round(to_thermal(corners, z)).astype(np.int32)],
                         float(celsius))
        thermal = cv2.resize(fine, (160, 120), interpolation=cv2.INTER_AREA)
        depths.append(depth)
        thermals.append(((thermal + 273.15) * 100).astype(np.uint16))

    return {
        "depth": np.stack(depths),
        "thermal": np.stack(thermals),
        "depth_k": depth_k.reshape(-1),
        "thermal_k": thermal_k.reshape(-1),
    }


def selftest(_arguments) -> int:
    frames = synthetic()
    prepared, thermal_k = prepare(frames)
    assert len(prepared) == len(frames["depth"]), "합성 프레임에서 경계가 안 나왔습니다"

    truth = np.array([*TRUTH_MM, 0.0, 0.0, 0.0])
    at_truth = score(truth, prepared, thermal_k, 1.5)
    for shift in (10.0, -10.0):
        moved = truth.copy()
        moved[0] += shift
        assert score(moved, prepared, thermal_k, 1.5) < at_truth, (
            f"{shift:+.0f} mm 밀었는데 점수가 안 떨어졌습니다 - 목적함수가 틀렸습니다")
    print(f"점수가 정답에서 최대  {at_truth:.4f}")

    start = truth.copy()
    start[0] += 12.0
    start[1] -= 8.0
    start[4] += 1.5
    found, _ = solve_extrinsic(prepared, thermal_k, start, quiet=True)
    error = np.linalg.norm(found[0:3] - TRUTH_MM)
    print(f"12/8 mm + 1.5 deg 밀어서 시작 -> 복원 오차 {error:.2f} mm, "
          f"회전 {np.linalg.norm(found[3:6]):.2f} deg")
    assert error < 3.0, f"복원 실패: {error:.2f} mm"
    print("selftest 통과")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    r = sub.add_parser("record", help="순찰 중 프레임 수집")
    r.add_argument("--out", default=str(RESULTS / "frames.npz"))
    r.add_argument("--frames", type=int, default=40)
    r.add_argument("--timeout", type=float, default=600.0)
    r.add_argument("--max-depth", type=float, default=3.0)
    r.add_argument("--min-move", type=float, default=0.25)
    r.add_argument("--append", action="store_true")
    r.add_argument("--depth-image", default="/depth_camera/image_raw")
    r.add_argument("--depth-info", default="/depth_camera/camera_info")
    r.add_argument("--thermal-image", default="/thermal_camera/image_raw")
    r.add_argument("--thermal-info", default="/thermal_camera/camera_info")
    r.add_argument("--odom", default="/odom")
    r.set_defaults(run=record)

    s = sub.add_parser("solve", help="수집한 프레임으로 외부파라미터 추정")
    s.add_argument("--frames", default=str(RESULTS / "frames.npz"))
    s.add_argument("--min-frames", type=int, default=5)
    s.add_argument("--start", type=float, nargs=6,
                   default=[*CAD_MM, 0.0, 0.0, 0.0],
                   help="초기값 tx ty tz mm, rx ry rz deg (기본값은 CAD 장착값)")
    s.add_argument("--start-at-truth", action="store_true")
    s.add_argument("--perturb", type=int, default=0,
                   help="초기값을 흔들어 N회 풀고 산포를 출력 (정답 없는 실기기용)")
    s.add_argument("--fx-error", type=float, default=0.0,
                   help="열화상 초점거리를 일부러 %% 만큼 틀리게 넣어 누출을 측정")
    s.set_defaults(run=solve)

    t = sub.add_parser("selftest", help="합성 데이터로 목적함수와 최적화 검사")
    t.set_defaults(run=selftest)

    arguments = parser.parse_args()
    return arguments.run(arguments)


if __name__ == "__main__":
    sys.exit(main())
