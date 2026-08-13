"""Solve the extrinsic by minimising reprojection error in both cameras.

This is the part of the paper that matters. The extrinsic is not measured and
it is not read off a drawing; it is whatever value makes the projected corners
land on the observed corners in both images at once, across every view.

The parameter vector holds the six extrinsic numbers and, alongside them, the
pose of the board in each view - six more per view. Those per-view poses are
not a side effect to be tolerated: the board's position is unknown in every
frame, and pretending otherwise by fixing it from RGB alone would push that
frame's RGB error into the extrinsic. Solving them jointly lets each view's
board settle wherever it best explains both cameras, and leaves the extrinsic
answering only for what the two cameras disagree about.

Intrinsics stay fixed, as in the paper. They are exact in simulation, and on
hardware they come from a separate single-camera run; solving for them here
would let intrinsic error hide inside the extrinsic - which is precisely what
happens, measurably, when the thermal focal length is wrong.

Mode A is the unconstrained six-degree-of-freedom fit. The constrained modes
build on the same residual and are added on top rather than replacing it.
"""
from __future__ import annotations

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

# What Gazebo actually renders, established by deriving it from the link
# origins and confirmed independently by the legacy circle-board run. Not the
# URDF optical-frame figure of (68, 0, 0): none of the <sensor> blocks carry a
# <pose>, so the optical-frame joints never reach the renderer. The optimiser
# never sees this - it is only for scoring at the end.
GROUND_TRUTH_MM = np.array([68.0, 12.0, -21.0])
GROUND_TRUTH_RPY_DEG = np.array([0.0, 0.0, 0.0])

# The mounting drawing, which is what a real robot would start from.
CAD_MM = np.array([68.0, 0.0, 0.0])


def rodrigues(vector: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(vector, np.float64).reshape(3, 1))[0]


def rpy_from_rotation(rotation: np.ndarray) -> np.ndarray:
    """Roll, pitch, yaw in degrees, in the fixed-axis convention URDF uses."""
    pitch = np.arcsin(-np.clip(rotation[2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.degrees([roll, pitch, yaw])


def pack(extrinsic: np.ndarray, poses: np.ndarray) -> np.ndarray:
    return np.concatenate([extrinsic, poses.reshape(-1)])


def unpack(params: np.ndarray, views: int):
    return params[:6], params[6:].reshape(views, 6)


def project(points: np.ndarray, rotation_vector, translation, k) -> np.ndarray:
    projected, _ = cv2.projectPoints(
        points, np.asarray(rotation_vector, np.float64).reshape(3, 1),
        np.asarray(translation, np.float64).reshape(3, 1), k, None)
    return projected.reshape(-1, 2)


def residuals(params, data) -> np.ndarray:
    """RGB and thermal reprojection errors, stacked, in pixels."""
    extrinsic, poses = unpack(params, data["views"])
    rotation_ext = rodrigues(extrinsic[3:6])
    out = []
    for index in range(data["views"]):
        rotation_vector, translation = poses[index, 0:3], poses[index, 3:6]
        rgb_projected = project(data["object_points"][index], rotation_vector,
                                translation, data["rgb_k"])
        out.append((rgb_projected - data["rgb_corners"][index]).reshape(-1))

        # The board in the thermal camera: through the board's own pose into
        # the RGB frame, then across the extrinsic.
        rotation_board = rodrigues(rotation_vector)
        rotation_thermal = rotation_ext @ rotation_board
        translation_thermal = rotation_ext @ np.asarray(translation) + extrinsic[0:3]
        tir_projected = project(
            data["object_matched"][index], cv2.Rodrigues(rotation_thermal)[0],
            translation_thermal, data["thermal_k"])
        out.append((tir_projected - data["tir_matched"][index]).reshape(-1))
    return np.concatenate(out)


def sparsity(data) -> lil_matrix:
    """Which residual depends on which parameter.

    Without this the solver probes all 6 + 6N parameters for every residual and
    the run goes from seconds to minutes. A view's board pose touches only that
    view's rows; the extrinsic touches only the thermal rows.
    """
    views = data["views"]
    rgb_rows = 2 * data["object_points"].shape[1]
    tir_rows = 2 * data["object_matched"].shape[1]
    total = views * (rgb_rows + tir_rows)
    matrix = lil_matrix((total, 6 + 6 * views), dtype=int)
    row = 0
    for index in range(views):
        matrix[row:row + rgb_rows, 6 + 6 * index:6 + 6 * index + 6] = 1
        row += rgb_rows
        matrix[row:row + tir_rows, 0:6] = 1
        matrix[row:row + tir_rows, 6 + 6 * index:6 + 6 * index + 6] = 1
        row += tir_rows
    return matrix


def initial_poses(data) -> np.ndarray:
    """Board pose per view from RGB alone, by PnP - the warm start."""
    poses = np.zeros((data["views"], 6))
    for index in range(data["views"]):
        ok, rotation_vector, translation = cv2.solvePnP(
            data["object_points"][index], data["rgb_corners"][index],
            data["rgb_k"], None,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise RuntimeError(f"뷰 {index} 의 PnP 실패")
        poses[index, 0:3] = rotation_vector.ravel()
        poses[index, 3:6] = translation.ravel()
    return poses


def initial_extrinsic(data, method: str) -> np.ndarray:
    """Six numbers to start from.

    'stereo' reproduces the paper, which warm-starts from OpenCV's closed-form
    stereo solution. 'cad' starts from the mounting drawing, which is what a
    real robot has before any calibration; it is offered so the two can be
    compared, because a method that only works from a good start is not much of
    a method.
    """
    if method == "cad":
        return np.concatenate([CAD_MM / 1000.0, np.zeros(3)])
    # OpenCV wants each view's points as its own (N, 1, C) float32 block. A
    # plain (N, C) array is rejected, and - less obviously - a list built as
    # [block] * views holds one array referenced many times, which the binding
    # collapses into a single view of N points.
    object_points = [np.ascontiguousarray(
        data["object_matched"][i].reshape(-1, 1, 3).astype(np.float32))
        for i in range(data["views"])]
    error, _, _, _, _, rotation, translation, _, _ = cv2.stereoCalibrate(
        object_points,
        [np.ascontiguousarray(p.reshape(-1, 1, 2).astype(np.float32))
         for p in data["rgb_matched"]],
        [np.ascontiguousarray(p.reshape(-1, 1, 2).astype(np.float32))
         for p in data["tir_matched"]],
        data["rgb_k"], None, data["thermal_k"], None,
        tuple(data["rgb_size"]), flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8))
    return np.concatenate([translation.ravel(), cv2.Rodrigues(rotation)[0].ravel()])


def split_errors(params, data):
    """Reprojection error broken out the way the paper reports it."""
    stacked = residuals(params, data)
    rgb_count = 2 * data["object_points"].shape[1]
    tir_count = 2 * data["object_matched"].shape[1]
    block = rgb_count + tir_count

    rgb_parts, tir_parts = [], []
    for index in range(data["views"]):
        start = index * block
        rgb_parts.append(stacked[start:start + rgb_count])
        tir_parts.append(stacked[start + rgb_count:start + block])
    rgb = np.stack(rgb_parts)
    tir = np.stack(tir_parts)

    def rms(values):
        return float(np.sqrt(np.mean(np.square(values))))

    # Per point: the same board corner across every view, so a corner that is
    # consistently worse than its neighbours points at the detector rather than
    # at the geometry.
    per_point_tir = np.sqrt(np.mean(
        tir.reshape(data["views"], -1, 2) ** 2, axis=(0, 2)))
    return {
        "rgb_rms_px": rms(rgb),
        "tir_rms_px": rms(tir),
        "total_rms_px": rms(stacked),
        "per_frame_rgb_px": [rms(row) for row in rgb],
        "per_frame_tir_px": [rms(row) for row in tir],
        "per_point_tir_px": per_point_tir.tolist(),
    }


def sensitivity(params, data, steps_mm=1.0, steps_deg=0.1):
    """How much thermal RMS moves when one extrinsic parameter is nudged.

    A parameter the data barely constrains shows a near-flat response here, and
    two that trade off against each other both look flatter than they are. It
    is the cheapest read on whether a calibration is well posed, and it costs
    twelve extra evaluations.

    Measured on the thermal residual, not the total. The RGB residual does not
    contain the extrinsic at all - it runs from the board pose straight to the
    RGB image - so with 77 RGB corners against 15 thermal ones, a total-RMS
    figure divides every real effect by roughly five and makes a
    well-determined parameter look as flat as an unobservable one.
    """
    labels = ["tx", "ty", "tz", "roll", "pitch", "yaw"]
    base = split_errors(params, data)["tir_rms_px"]
    rows = []
    for axis in range(6):
        step = steps_mm / 1000.0 if axis < 3 else np.radians(steps_deg)
        moved = np.array(params, np.float64)
        moved[axis] += step
        after = split_errors(moved, data)["tir_rms_px"]
        rows.append({
            "parameter": labels[axis],
            "step": f"+{steps_mm:g}mm" if axis < 3 else f"+{steps_deg:g}deg",
            "delta_tir_rms_px": round(after - base, 4),
        })
    return base, rows


def solve(data, init: str = "stereo", verbose: bool = True):
    poses = initial_poses(data)
    extrinsic = initial_extrinsic(data, init)
    start = pack(extrinsic, poses)

    result = least_squares(
        residuals, start, jac_sparsity=sparsity(data), x_scale="jac",
        method="trf", loss="linear", ftol=1e-12, xtol=1e-12, gtol=1e-12,
        max_nfev=400, args=(data,), verbose=0)

    extrinsic, _ = unpack(result.x, data["views"])
    rotation = rodrigues(extrinsic[3:6])
    report = {
        "translation_mm": (extrinsic[0:3] * 1000.0).tolist(),
        "rpy_deg": rpy_from_rotation(rotation).tolist(),
        "init": init,
        "iterations": int(result.nfev),
    }
    report.update(split_errors(result.x, data))
    if verbose:
        before = split_errors(start, data)
        print(f"  초기값({init})  전체 RMS {before['total_rms_px']:.3f} px")
        print(f"  최적화 후      전체 RMS {report['total_rms_px']:.3f} px "
              f"(RGB {report['rgb_rms_px']:.3f} / 열화상 {report['tir_rms_px']:.3f})")
    return result.x, report
