#!/usr/bin/env python3
"""Interactively calibrate a physical RGB-D camera against a thermal camera.

SPACE captures a detected 6 x 4-square board pose (5 x 3 inner corners), S
selects the useful views, solves and saves, D removes the last sample, and Q
exits. RGB and thermal corners solve the geometry; depth is retained with every
sample to verify the HP60C's manufacturer registration.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import select
import shutil
import sys
import termios
import time
import tty

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


DEFAULT_ROOT = Path("~/.local/share/hazard_guard/calibration").expanduser()
PATTERN_COLUMNS = 5
PATTERN_ROWS = 3
SQUARE_SIZE_M = 0.0475
THERMAL_LOW_PERCENTILE = 1.0
THERMAL_HIGH_PERCENTILE = 99.0
# Three panels are shown side by side.  Keep the combined 1440 px width inside
# the Jetson desktop instead of creating the previous 1800 px-wide window.
PREVIEW_WIDTH = 480
PREVIEW_HEIGHT = 360
PREVIEW_INTERVAL_SEC = 0.25
WINDOW_NAME = "HazardGuard RGB-D / Thermal Calibration"


@dataclass
class ReceivedImage:
    receipt_time: float
    message: Image


@dataclass
class CapturedView:
    index: int
    path: Path
    rgb_corners: np.ndarray
    thermal_corners: np.ndarray
    object_points: np.ndarray
    rgb_size: tuple[int, int]
    thermal_size: tuple[int, int]
    rgb_frame: str
    thermal_frame: str
    relative_rotation: np.ndarray
    relative_translation: np.ndarray


@dataclass
class ViewSelectionMetric:
    view: CapturedView
    cell: tuple[int, int]
    feature: np.ndarray


def corner_rms(first: np.ndarray, second: np.ndarray) -> float:
    """Return the RMS corner displacement in pixels."""
    if first.shape != second.shape:
        return math.inf
    return float(np.sqrt(np.mean((first.astype(float) - second) ** 2)))


def rotation_distance(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular distance between two rotation matrices."""
    cosine = np.clip((np.trace(first.T @ second) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(cosine))


def image_grid_cell(
    corners: np.ndarray, image_size: tuple[int, int]
) -> tuple[int, int]:
    """Return the 3x3 grid cell containing the board's corner centroid."""
    centre = np.mean(corners, axis=0)
    width, height = image_size
    return (
        min(2, max(0, int(centre[0] * 3.0 / width))),
        min(2, max(0, int(centre[1] * 3.0 / height))),
    )


def draw_thirds_grid(image: np.ndarray) -> None:
    """Draw a display-only 3x3 capture guide over the thermal preview."""
    height, width = image.shape[:2]
    color = (180, 180, 180)
    for fraction in (1, 2):
        cv2.line(
            image, (width * fraction // 3, 0),
            (width * fraction // 3, height - 1), color, 1, cv2.LINE_AA,
        )
        cv2.line(
            image, (0, height * fraction // 3),
            (width - 1, height * fraction // 3), color, 1, cv2.LINE_AA,
        )
    for row in range(3):
        for column in range(3):
            cv2.putText(
                image,
                str(row * 3 + column + 1),
                (column * width // 3 + 3, row * height // 3 + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (235, 235, 235),
                1,
                cv2.LINE_AA,
            )


def estimate_relative_pose(
    object_points: np.ndarray,
    rgb_corners: np.ndarray,
    thermal_corners: np.ndarray,
    rgb_info: CameraInfo,
    thermal_info: CameraInfo,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate X_thermal = R * X_rgb + t from one board observation."""
    rgb_ok, rgb_rvec, rgb_tvec = cv2.solvePnP(
        object_points, rgb_corners, camera_matrix(rgb_info), distortion(rgb_info)
    )
    thermal_ok, thermal_rvec, thermal_tvec = cv2.solvePnP(
        object_points,
        thermal_corners,
        camera_matrix(thermal_info),
        distortion(thermal_info),
    )
    if not rgb_ok or not thermal_ok:
        raise ValueError("체커보드 포즈를 계산할 수 없습니다")
    rgb_rotation = cv2.Rodrigues(rgb_rvec)[0]
    thermal_rotation = cv2.Rodrigues(thermal_rvec)[0]
    rotation = thermal_rotation @ rgb_rotation.T
    translation = (
        thermal_tvec.reshape(3) - rotation @ rgb_tvec.reshape(3)
    )
    return rotation, translation


def pose_outlier_reason(
    rotation: np.ndarray,
    translation: np.ndarray,
    previous: list[CapturedView],
    minimum_reference_views: int = 8,
) -> str | None:
    """Reject only gross online pose outliers using robust prior statistics."""
    if len(previous) < minimum_reference_views:
        return None
    translations = np.asarray([view.relative_translation for view in previous])
    translation_centre = np.median(translations, axis=0)
    rotation_centre = min(
        (view.relative_rotation for view in previous),
        key=lambda candidate: sum(
            rotation_distance(candidate, view.relative_rotation)
            for view in previous
        ),
    )
    translation_errors = np.linalg.norm(
        translations - translation_centre, axis=1
    )
    rotation_errors = np.asarray([
        rotation_distance(rotation_centre, view.relative_rotation)
        for view in previous
    ])

    def robust_limit(errors: np.ndarray, floor: float) -> float:
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        return max(floor, median + 4.0 * 1.4826 * mad)

    translation_error = float(np.linalg.norm(translation - translation_centre))
    rotation_error = rotation_distance(rotation_centre, rotation)
    translation_limit = robust_limit(translation_errors, 0.08)
    rotation_limit = robust_limit(rotation_errors, math.radians(10.0))
    if translation_error > translation_limit:
        return (
            f"T 이상치 {translation_error * 1000:.1f} mm > "
            f"{translation_limit * 1000:.1f} mm"
        )
    if rotation_error > rotation_limit:
        return (
            f"R 이상치 {math.degrees(rotation_error):.1f} deg > "
            f"{math.degrees(rotation_limit):.1f} deg"
        )
    return None


def global_pose_outlier_indices(views: list[CapturedView]) -> set[int]:
    """Find gross relative-pose outliers after all captures are available."""
    if len(views) < 8:
        return set()
    translations = np.asarray([view.relative_translation for view in views])
    translation_centre = np.median(translations, axis=0)
    rotation_centre = min(
        (view.relative_rotation for view in views),
        key=lambda candidate: sum(
            rotation_distance(candidate, view.relative_rotation)
            for view in views
        ),
    )
    translation_errors = np.linalg.norm(
        translations - translation_centre, axis=1
    )
    rotation_errors = np.asarray([
        rotation_distance(rotation_centre, view.relative_rotation)
        for view in views
    ])

    def limit(errors: np.ndarray, floor: float) -> float:
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        return max(floor, median + 4.0 * 1.4826 * mad)

    translation_limit = limit(translation_errors, 0.08)
    rotation_limit = limit(rotation_errors, math.radians(10.0))
    return {
        view.index
        for view, translation_error, rotation_error in zip(
            views, translation_errors, rotation_errors
        )
        if translation_error > translation_limit or rotation_error > rotation_limit
    }


def selection_metric(
    view: CapturedView, rgb_info: CameraInfo
) -> ViewSelectionMetric:
    """Describe image position, distance and board orientation for selection."""
    ok, rotation_vector, translation = cv2.solvePnP(
        view.object_points,
        view.rgb_corners,
        camera_matrix(rgb_info),
        distortion(rgb_info),
    )
    if not ok:
        raise ValueError(f"view {view.index:03d} RGB 포즈 계산 실패")
    rotation = cv2.Rodrigues(rotation_vector)[0]
    normal = rotation[:, 2]
    tilt_x = math.atan2(float(normal[0]), float(normal[2]))
    tilt_y = math.atan2(float(normal[1]), float(normal[2]))
    direction = view.thermal_corners[4] - view.thermal_corners[0]
    in_plane = math.atan2(float(direction[1]), float(direction[0]))
    centre = np.mean(view.thermal_corners, axis=0)
    width, height = view.thermal_size
    cell = image_grid_cell(view.thermal_corners, view.thermal_size)
    # Unit scales make one grid cell, 15 degrees of tilt, and 15 cm of depth
    # similarly important to farthest-point diversity selection.
    feature = np.asarray([
        centre[0] * 3.0 / width,
        centre[1] * 3.0 / height,
        float(translation.reshape(3)[2]) / 0.15,
        tilt_x / math.radians(15.0),
        tilt_y / math.radians(15.0),
        math.sin(2.0 * in_plane),
        math.cos(2.0 * in_plane),
    ])
    return ViewSelectionMetric(view=view, cell=cell, feature=feature)


def select_diverse_views(
    views: list[CapturedView],
    rgb_info: CameraInfo,
    maximum_views: int,
    duplicate_rgb_rms_px: float = 6.0,
    duplicate_thermal_rms_px: float = 2.0,
) -> tuple[list[CapturedView], list[dict], dict[str, int]]:
    """Reject duplicates/outliers, then select diverse 3x3-grid board poses."""
    unique_views: list[CapturedView] = []
    rejected: list[dict] = []
    for view in views:
        duplicate = next((
            previous for previous in unique_views
            if corner_rms(view.rgb_corners, previous.rgb_corners)
            < duplicate_rgb_rms_px
            and corner_rms(view.thermal_corners, previous.thermal_corners)
            < duplicate_thermal_rms_px
        ), None)
        if duplicate is not None:
            rejected.append({
                "index": view.index,
                "reason": f"duplicate_of_view_{duplicate.index:03d}",
            })
            continue
        unique_views.append(view)

    gross_outliers = global_pose_outlier_indices(unique_views)
    rejected.extend([
        {"index": view.index, "reason": "gross_relative_pose_outlier"}
        for view in unique_views if view.index in gross_outliers
    ])
    metrics = [
        selection_metric(view, rgb_info)
        for view in unique_views if view.index not in gross_outliers
    ]
    if len(metrics) <= maximum_views:
        cells = {metric.cell for metric in metrics}
        return [metric.view for metric in metrics], rejected, {
            "occupied_cells": len(cells), "total_cells": 9,
        }

    by_cell: dict[tuple[int, int], list[ViewSelectionMetric]] = {}
    for metric in metrics:
        by_cell.setdefault(metric.cell, []).append(metric)

    selected: list[ViewSelectionMetric] = []
    # Seed every occupied image cell with its most representative observation.
    for cell in sorted(by_cell):
        group = by_cell[cell]
        centre = np.median(np.asarray([item.feature for item in group]), axis=0)
        selected.append(min(group, key=lambda item: np.linalg.norm(item.feature - centre)))
    selected = selected[:maximum_views]
    selected_indices = {item.view.index for item in selected}
    cell_limit = max(1, math.ceil(maximum_views / max(1, len(by_cell))))

    while len(selected) < maximum_views:
        cell_counts = {
            cell: sum(item.cell == cell for item in selected) for cell in by_cell
        }
        candidates = [
            item for item in metrics
            if item.view.index not in selected_indices
            and cell_counts[item.cell] < cell_limit
        ]
        if not candidates:
            candidates = [
                item for item in metrics
                if item.view.index not in selected_indices
            ]
        if not candidates:
            break
        chosen = max(
            candidates,
            key=lambda item: min(
                np.linalg.norm(item.feature - existing.feature)
                for existing in selected
            ),
        )
        selected.append(chosen)
        selected_indices.add(chosen.view.index)

    for metric in metrics:
        if metric.view.index not in selected_indices:
            rejected.append({
                "index": metric.view.index,
                "reason": "diversity_limit",
                "grid_cell": list(metric.cell),
            })
    selected.sort(key=lambda item: item.view.index)
    return [item.view for item in selected], rejected, {
        "occupied_cells": len(by_cell), "total_cells": 9,
    }


def message_stamp_seconds(message: Image) -> float:
    return float(message.header.stamp.sec) + message.header.stamp.nanosec * 1.0e-9


def board_object_points(
    columns: int = PATTERN_COLUMNS,
    rows: int = PATTERN_ROWS,
    square_size_m: float = SQUARE_SIZE_M,
) -> np.ndarray:
    """Return row-major inner corners on a planar board, in metres."""
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= square_size_m
    return points


def normalize_thermal(thermal: np.ndarray) -> np.ndarray:
    """Turn radiometric mono16 into robust detector input without altering it."""
    values = thermal.astype(np.float32)
    low, high = np.percentile(
        values, [THERMAL_LOW_PERCENTILE, THERMAL_HIGH_PERCENTILE]
    )
    if high - low < 1.0e-6:
        return np.zeros(values.shape, dtype=np.uint8)
    scaled = np.clip((values - low) / (high - low), 0.0, 1.0)
    return (scaled * 255.0).astype(np.uint8)


def thermal_preview(thermal: np.ndarray) -> np.ndarray:
    """Render radiometric thermal data without modifying saved values."""
    return cv2.applyColorMap(normalize_thermal(thermal), cv2.COLORMAP_INFERNO)


def depth_preview(depth: np.ndarray, encoding: str) -> tuple[np.ndarray, str]:
    """Create a robust, coloured depth preview and its visible metric range."""
    values = depth.astype(np.float32)
    if encoding.upper() in ("16UC1", "MONO16"):
        values *= 0.001
    valid = np.isfinite(values) & (values > 0.0)
    display = np.zeros(values.shape, dtype=np.uint8)
    if not np.any(valid):
        return cv2.applyColorMap(display, cv2.COLORMAP_TURBO), "no valid depth"
    low, high = np.percentile(values[valid], [2.0, 98.0])
    if high - low < 1.0e-6:
        high = low + 1.0e-6
    display[valid] = (
        np.clip((values[valid] - low) / (high - low), 0.0, 1.0) * 255.0
    ).astype(np.uint8)
    coloured = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)
    coloured[~valid] = 0
    return coloured, f"{low:.2f}-{high:.2f} m"


def preview_panel(
    image: np.ndarray,
    title: str,
    detail: str,
    width: int = PREVIEW_WIDTH,
    height: int = PREVIEW_HEIGHT,
) -> np.ndarray:
    """Letterbox an image and add readable preview labels."""
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    available_height = height - 54
    scale = min(width / image.shape[1], available_height / image.shape[0])
    resized = cv2.resize(
        image,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_NEAREST if scale > 1.0 else cv2.INTER_AREA,
    )
    x = (width - resized.shape[1]) // 2
    y = 30 + (available_height - resized.shape[0]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    cv2.putText(
        canvas, title, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, detail, (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX,
        0.50, (230, 230, 230), 1, cv2.LINE_AA,
    )
    return canvas


def find_corners(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    found, corners = cv2.findChessboardCornersSB(
        gray,
        pattern,
        cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
    )
    if not found:
        return None
    return corners.reshape(-1, 2).astype(np.float32)


def detect_rgb(image: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return find_corners(gray, pattern)


def detect_thermal(
    image: np.ndarray, pattern: tuple[int, int]
) -> tuple[np.ndarray | None, str]:
    gray = normalize_thermal(image)
    corners = find_corners(gray, pattern)
    if corners is not None:
        return corners, "native"
    scale = 4
    enlarged = cv2.resize(
        gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
    )
    corners = find_corners(enlarged, pattern)
    if corners is not None:
        return corners / scale, "upsampled"
    equalized = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(
        enlarged
    )
    corners = find_corners(equalized, pattern)
    if corners is not None:
        return corners / scale, "clahe"
    return None, "none"


def align_corner_order(rgb: np.ndarray, thermal: np.ndarray) -> np.ndarray:
    """Resolve the independent 180-degree ordering of a symmetric board."""
    rgb_direction = rgb[-1] - rgb[0]
    thermal_direction = thermal[-1] - thermal[0]
    if float(np.dot(rgb_direction, thermal_direction)) < 0.0:
        return thermal[::-1].copy()
    return thermal


def camera_matrix(info: CameraInfo) -> np.ndarray:
    matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("CameraInfo has invalid focal lengths")
    return matrix


def distortion(info: CameraInfo) -> np.ndarray:
    values = np.asarray(info.d, dtype=np.float64)
    return values.reshape(-1, 1) if values.size else np.zeros((5, 1))


def invert_transform(
    rotation_target_from_source: np.ndarray,
    translation_target_from_source: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Invert X_target = R * X_source + t."""
    rotation_source_from_target = rotation_target_from_source.T
    translation_source_from_target = (
        -rotation_source_from_target @ translation_target_from_source.reshape(3)
    )
    return rotation_source_from_target, translation_source_from_target


def quaternion_from_rotation(rotation: np.ndarray) -> list[float]:
    """Return a normalized ROS-order quaternion [x, y, z, w]."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
            w = (matrix[2, 1] - matrix[1, 2]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
            w = (matrix[0, 2] - matrix[2, 0]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
            w = (matrix[1, 0] - matrix[0, 1]) / scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion.tolist()


def camera_info_dict(info: CameraInfo) -> dict:
    return {
        "width": int(info.width),
        "height": int(info.height),
        "distortion_model": info.distortion_model,
        "d": list(info.d),
        "k": list(info.k),
        "r": list(info.r),
        "p": list(info.p),
        "frame_id": info.header.frame_id,
    }


def write_with_backup(path: Path, text: str) -> Path | None:
    path.parent.mkdir(parents=True, exist_ok=True)
    backup = None
    if path.exists():
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.stem}.{stamp}.bak{path.suffix}")
        shutil.copy2(path, backup)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(temporary, path)
    return backup


def thermal_camera_yaml(
    width: int,
    height: int,
    matrix: np.ndarray,
    coefficients: np.ndarray,
) -> str:
    distortion_values = coefficients.reshape(-1).tolist()
    projection = [
        float(matrix[0, 0]), 0.0, float(matrix[0, 2]), 0.0,
        0.0, float(matrix[1, 1]), float(matrix[1, 2]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    document = {
        "image_width": width,
        "image_height": height,
        "camera_name": "thermal_camera",
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix.reshape(-1).tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(distortion_values),
            "data": distortion_values,
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).reshape(-1).tolist(),
        },
        "projection_matrix": {"rows": 3, "cols": 4, "data": projection},
    }
    return yaml.safe_dump(document, sort_keys=False)


def calibrate_view_set(
    views: list[CapturedView], rgb_info: CameraInfo
) -> tuple[float, np.ndarray, np.ndarray, float, np.ndarray, np.ndarray]:
    """Calibrate intrinsics and RGB-to-thermal extrinsics for selected views."""
    objects = [view.object_points.reshape(-1, 1, 3) for view in views]
    rgb_points = [view.rgb_corners.reshape(-1, 1, 2) for view in views]
    thermal_points = [
        view.thermal_corners.reshape(-1, 1, 2) for view in views
    ]
    thermal_size = views[0].thermal_size
    thermal_guess = cv2.initCameraMatrix2D(
        objects, thermal_points, thermal_size
    )
    thermal_rms, thermal_k, thermal_d, _, _ = cv2.calibrateCamera(
        objects,
        thermal_points,
        thermal_size,
        thermal_guess,
        None,
        flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3,
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            100,
            1.0e-8,
        ),
    )
    rgb_k = camera_matrix(rgb_info)
    rgb_d = distortion(rgb_info)
    stereo_rms, _, _, _, _, rotation, translation, _, _ = cv2.stereoCalibrate(
        objects,
        rgb_points,
        thermal_points,
        rgb_k.copy(),
        rgb_d.copy(),
        thermal_k.copy(),
        thermal_d.copy(),
        views[0].rgb_size,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            100,
            1.0e-8,
        ),
    )
    return (
        float(thermal_rms), thermal_k, thermal_d,
        float(stereo_rms), rotation, translation,
    )


def view_reprojection_errors(
    views: list[CapturedView],
    rgb_info: CameraInfo,
    thermal_k: np.ndarray,
    thermal_d: np.ndarray,
    thermal_from_rgb: np.ndarray,
    thermal_translation: np.ndarray,
) -> dict[int, float]:
    """Measure each selected view against the solved cross-camera geometry."""
    rgb_k = camera_matrix(rgb_info)
    rgb_d = distortion(rgb_info)
    errors = {}
    for view in views:
        ok, rotation_vector, translation = cv2.solvePnP(
            view.object_points, view.rgb_corners, rgb_k, rgb_d
        )
        if not ok:
            errors[view.index] = math.inf
            continue
        board_from_rgb = cv2.Rodrigues(rotation_vector)[0]
        points_rgb = (
            board_from_rgb @ view.object_points.astype(float).T
            + translation.reshape(3, 1)
        ).T
        points_thermal = (
            thermal_from_rgb @ points_rgb.T
            + thermal_translation.reshape(3, 1)
        ).T
        projected, _ = cv2.projectPoints(
            points_thermal,
            np.zeros(3),
            np.zeros(3),
            thermal_k,
            thermal_d,
        )
        residual = projected.reshape(-1, 2) - view.thermal_corners
        errors[view.index] = float(
            np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))
        )
    return errors


def reprojection_outlier_limit(
    errors: dict[int, float], maximum_rms_px: float
) -> float:
    """Combine a robust distribution limit with an absolute pixel ceiling."""
    finite = np.asarray([value for value in errors.values() if np.isfinite(value)])
    if not finite.size:
        return maximum_rms_px
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    robust = max(2.0, median + 3.0 * 1.4826 * mad)
    return min(maximum_rms_px, robust)


class TerminalKeys:
    def __init__(self) -> None:
        self.fd: int | None = None
        self.attributes = None
        self.flags: int | None = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("interactive calibration requires a terminal")
        self.fd = sys.stdin.fileno()
        self.attributes = termios.tcgetattr(self.fd)
        self.flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        tty.setcbreak(self.fd)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.flags | os.O_NONBLOCK)
        return self

    def read(self) -> str | None:
        if self.fd is None or not select.select([self.fd], [], [], 0.0)[0]:
            return None
        return os.read(self.fd, 1).decode("utf-8", errors="ignore")

    def __exit__(self, *_args) -> None:
        if self.fd is not None and self.attributes is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.attributes)
        if self.fd is not None and self.flags is not None:
            fcntl.fcntl(self.fd, fcntl.F_SETFL, self.flags)


class PhysicalThermalCalibration(Node):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__("physical_thermal_calibration")
        self.arguments = arguments
        self.bridge = CvBridge()
        self.rgb: deque[ReceivedImage] = deque(maxlen=40)
        self.depth: deque[ReceivedImage] = deque(maxlen=40)
        self.thermal: deque[ReceivedImage] = deque(maxlen=40)
        self.rgb_info: CameraInfo | None = None
        self.depth_info: CameraInfo | None = None
        self.thermal_info: CameraInfo | None = None
        self.views: list[CapturedView] = []
        self.preview_enabled = not arguments.no_gui
        self.last_preview_time = 0.0
        self.preview_opened = False
        self.last_auto_attempt = -math.inf
        self.stable_since: float | None = None
        self.last_detection_time: float | None = None
        self.last_detected_rgb: np.ndarray | None = None
        self.last_detected_thermal: np.ndarray | None = None
        self.pattern = (arguments.pattern_columns, arguments.pattern_rows)
        self.points = board_object_points(
            arguments.pattern_columns,
            arguments.pattern_rows,
            arguments.square_size_m,
        )
        session_name = datetime.now().strftime("physical-%Y%m%d-%H%M%S")
        self.session = arguments.output_dir / "sessions" / session_name
        self.session.mkdir(parents=True, exist_ok=False)

        self.create_subscription(
            Image, arguments.rgb_topic,
            lambda message: self._image(self.rgb, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, arguments.depth_topic,
            lambda message: self._image(self.depth, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, arguments.thermal_topic,
            lambda message: self._image(self.thermal, message),
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
        self.create_subscription(
            CameraInfo, arguments.thermal_info_topic,
            lambda message: setattr(self, "thermal_info", message),
            qos_profile_sensor_data,
        )

    def update_preview(self) -> str | None:
        """Refresh the three-camera preview and return an OpenCV key press."""
        if not self.preview_enabled:
            return None
        now = time.monotonic()
        if now - self.last_preview_time >= PREVIEW_INTERVAL_SEC:
            self.last_preview_time = now
            waiting = np.zeros((240, 320, 3), dtype=np.uint8)
            panels = []
            preview_rgb_corners = None
            preview_thermal_corners = None

            if self.rgb:
                rgb_message = self.rgb[-1].message
                rgb = self.bridge.imgmsg_to_cv2(
                    rgb_message, desired_encoding="bgr8"
                )
                rgb_corners = detect_rgb(rgb, self.pattern)
                preview_rgb_corners = rgb_corners
                rgb_view = rgb.copy()
                if rgb_corners is not None:
                    cv2.drawChessboardCorners(
                        rgb_view, self.pattern,
                        rgb_corners.reshape(-1, 1, 2), True,
                    )
                rgb_detail = (
                    "corners OK - ready" if rgb_corners is not None
                    else "corners NOT found"
                )
                panels.append(preview_panel(rgb_view, "RGB", rgb_detail))
            else:
                panels.append(preview_panel(waiting, "RGB", "waiting for topic"))

            if self.depth:
                depth_message = self.depth[-1].message
                depth = self.bridge.imgmsg_to_cv2(
                    depth_message, desired_encoding="passthrough"
                )
                depth_view, depth_range = depth_preview(
                    depth, depth_message.encoding
                )
                panels.append(
                    preview_panel(depth_view, "DEPTH", depth_range)
                )
            else:
                panels.append(
                    preview_panel(waiting, "DEPTH", "waiting for topic")
                )

            if self.thermal:
                thermal_message = self.thermal[-1].message
                thermal = self.bridge.imgmsg_to_cv2(
                    thermal_message, desired_encoding="passthrough"
                )
                thermal_view = thermal_preview(thermal)
                draw_thirds_grid(thermal_view)
                thermal_corners, method = detect_thermal(
                    thermal, self.pattern
                )
                preview_thermal_corners = thermal_corners
                if thermal_corners is not None:
                    cv2.drawChessboardCorners(
                        thermal_view, self.pattern,
                        thermal_corners.reshape(-1, 1, 2), True,
                    )
                thermal_detail = (
                    f"corners OK ({method}) - ready"
                    if thermal_corners is not None
                    else "corners NOT found"
                )
                panels.append(
                    preview_panel(thermal_view, "THERMAL", thermal_detail)
                )
            else:
                panels.append(
                    preview_panel(waiting, "THERMAL", "waiting for topic")
                )

            ready, _ = self.readiness()
            if (
                ready
                and preview_rgb_corners is not None
                and preview_thermal_corners is not None
            ):
                self.observe_detection(
                    now,
                    preview_rgb_corners,
                    align_corner_order(
                        preview_rgb_corners, preview_thermal_corners
                    ),
                )
            else:
                self.reset_stability()

            body = np.hstack(panels)
            footer = np.full((58, body.shape[1], 3), 18, dtype=np.uint8)
            ready, missing = self.readiness()
            state = "topics OK" if ready else "waiting: " + ", ".join(missing)
            occupied_cells = {
                image_grid_cell(view.thermal_corners, view.thermal_size)
                for view in self.views
            }
            state += f" | grid {len(occupied_cells)}/9"
            if self.arguments.auto_capture_interval_sec > 0.0:
                held = self.stable_duration(now)
                state += (
                    f" | AUTO {self.arguments.auto_capture_interval_sec:.1f}s"
                    f" hold {min(held, self.arguments.stable_duration_sec):.1f}/"
                    f"{self.arguments.stable_duration_sec:.1f}s"
                )
            cv2.putText(
                footer,
                f"SPACE capture | D delete | S solve/save | Q quit"
                f"     saved: {len(self.views)}     {state}",
                (14, 37), cv2.FONT_HERSHEY_SIMPLEX,
                0.50, (245, 245, 245), 1, cv2.LINE_AA,
            )
            preview = np.vstack((body, footer))
            if not self.preview_opened:
                cv2.namedWindow(
                    WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO
                )
                cv2.resizeWindow(
                    WINDOW_NAME, preview.shape[1], preview.shape[0]
                )
            cv2.imshow(WINDOW_NAME, preview)
            self.preview_opened = True

        key_code = cv2.waitKey(1) & 0xFF
        if key_code in (255,):
            return None
        if key_code == 27:
            return "q"
        return chr(key_code)

    def reset_stability(self) -> None:
        self.stable_since = None
        self.last_detection_time = None
        self.last_detected_rgb = None
        self.last_detected_thermal = None

    def observe_detection(
        self,
        now: float,
        rgb_corners: np.ndarray,
        thermal_corners: np.ndarray,
    ) -> None:
        continuous = (
            self.last_detection_time is not None
            and now - self.last_detection_time <= 2.0 * PREVIEW_INTERVAL_SEC
        )
        stable = (
            continuous
            and self.last_detected_rgb is not None
            and self.last_detected_thermal is not None
            and corner_rms(rgb_corners, self.last_detected_rgb)
            <= self.arguments.stability_rgb_rms_px
            and corner_rms(thermal_corners, self.last_detected_thermal)
            <= self.arguments.stability_thermal_rms_px
        )
        if not stable:
            self.stable_since = now
        self.last_detection_time = now
        self.last_detected_rgb = rgb_corners.copy()
        self.last_detected_thermal = thermal_corners.copy()

    def stable_duration(self, now: float | None = None) -> float:
        if self.stable_since is None:
            return 0.0
        return max(0.0, (now or time.monotonic()) - self.stable_since)

    def maybe_auto_capture(self) -> bool:
        interval = self.arguments.auto_capture_interval_sec
        if interval <= 0.0:
            return False
        now = time.monotonic()
        if now - self.last_auto_attempt < interval:
            return False
        self.last_auto_attempt = now
        if self.stable_duration(now) < self.arguments.stable_duration_sec:
            return False
        return self.capture(automatic=True)

    def close_preview(self) -> None:
        if self.preview_opened:
            cv2.destroyWindow(WINDOW_NAME)
            cv2.waitKey(1)
            self.preview_opened = False

    @staticmethod
    def _image(queue: deque[ReceivedImage], message: Image) -> None:
        queue.append(ReceivedImage(time.monotonic(), message))

    def readiness(self) -> tuple[bool, list[str]]:
        missing = []
        for name in ("rgb", "depth", "thermal"):
            queue = getattr(self, name)
            if not queue or time.monotonic() - queue[-1].receipt_time > 2.0:
                missing.append(name)
        for name in ("rgb_info", "depth_info", "thermal_info"):
            if getattr(self, name) is None:
                missing.append(name)
        return not missing, missing

    @staticmethod
    def _closest(
        queue: deque[ReceivedImage], receipt_time: float
    ) -> ReceivedImage:
        return min(queue, key=lambda item: abs(item.receipt_time - receipt_time))

    def capture(self, automatic: bool = False) -> bool:
        ready, missing = self.readiness()
        if not ready:
            print(f"\n캡처 불가: 수신 대기 중 ({', '.join(missing)})")
            return False
        if (
            automatic
            and self.preview_enabled
            and self.stable_duration() < self.arguments.stable_duration_sec
        ):
            print(
                "\n캡처 불가: 체커보드를 "
                f"{self.arguments.stable_duration_sec:.1f}초 이상 정지하세요"
            )
            return False

        thermal_received = self.thermal[-1]
        rgb_received = self._closest(self.rgb, thermal_received.receipt_time)
        depth_received = self._closest(self.depth, thermal_received.receipt_time)
        receipt_delta = abs(
            rgb_received.receipt_time - thermal_received.receipt_time
        )
        if (
            automatic
            and receipt_delta > self.arguments.max_receipt_offset_sec
        ):
            print(
                f"\n{'자동 건너뜀' if automatic else '캡처 불가'}: "
                f"RGB/Thermal 수신 시각차 {receipt_delta:.3f}s"
            )
            return False
        rgb_message = rgb_received.message
        depth_message = depth_received.message
        thermal_message = thermal_received.message

        rgb = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        thermal = self.bridge.imgmsg_to_cv2(
            thermal_message, desired_encoding="passthrough"
        )
        if thermal.ndim != 2:
            print("\n캡처 불가: thermal 토픽은 mono16 원본이어야 합니다")
            return False

        rgb_corners = detect_rgb(rgb, self.pattern)
        thermal_corners, thermal_method = detect_thermal(thermal, self.pattern)
        if rgb_corners is None or thermal_corners is None:
            print(
                "\n검출 실패: "
                f"RGB={'성공' if rgb_corners is not None else '실패'}, "
                f"Thermal={thermal_method}. 판을 정지하고 위치·각도를 조정하세요."
            )
            return False
        thermal_corners = align_corner_order(rgb_corners, thermal_corners)

        if automatic:
            thermal_area = float(
                cv2.contourArea(cv2.convexHull(thermal_corners))
            )
            margin = self.arguments.thermal_border_margin_px
            inside_margin = bool(
                np.all(thermal_corners[:, 0] >= margin)
                and np.all(thermal_corners[:, 0] < thermal.shape[1] - margin)
                and np.all(thermal_corners[:, 1] >= margin)
                and np.all(thermal_corners[:, 1] < thermal.shape[0] - margin)
            )
            if thermal_area < self.arguments.min_thermal_board_area_px2:
                print(
                    "\n자동 건너뜀: 열화상 보드가 너무 작거나 기울어짐 "
                    f"({thermal_area:.0f}px²)"
                )
                return False
            if not inside_margin:
                print("\n자동 건너뜀: 열화상 코너가 화면 가장자리에 너무 가까움")
                return False

            for view in self.views:
                if (
                    corner_rms(rgb_corners, view.rgb_corners)
                    < self.arguments.duplicate_rgb_rms_px
                    and corner_rms(thermal_corners, view.thermal_corners)
                    < self.arguments.duplicate_thermal_rms_px
                ):
                    print(
                        "\n자동 건너뜀: "
                        f"view {view.index:03d}과 너무 비슷한 자세"
                    )
                    return False

        try:
            relative_rotation, relative_translation = estimate_relative_pose(
                self.points,
                rgb_corners,
                thermal_corners,
                self.rgb_info,
                self.thermal_info,
            )
        except (ValueError, cv2.error) as error:
            print(
                f"\n{'자동 건너뜀' if automatic else '캡처 불가'}: {error}"
            )
            return False
        if automatic:
            outlier = pose_outlier_reason(
                relative_rotation, relative_translation, self.views
            )
            if outlier is not None:
                print(f"\n자동 건너뜀: {outlier}")
                return False

        index = len(self.views) + 1
        path = self.session / f"view_{index:03d}.npz"
        np.savez_compressed(
            path,
            rgb=rgb,
            depth=depth,
            thermal=thermal,
            rgb_corners=rgb_corners,
            thermal_corners=thermal_corners,
            object_points=self.points,
            rgb_stamp=message_stamp_seconds(rgb_message),
            depth_stamp=message_stamp_seconds(depth_message),
            thermal_stamp=message_stamp_seconds(thermal_message),
            rgb_receipt_offset_sec=(
                rgb_received.receipt_time - thermal_received.receipt_time
            ),
            depth_receipt_offset_sec=(
                depth_received.receipt_time - thermal_received.receipt_time
            ),
            rgb_encoding=rgb_message.encoding,
            depth_encoding=depth_message.encoding,
            thermal_encoding=thermal_message.encoding,
            rgb_frame=rgb_message.header.frame_id,
            depth_frame=depth_message.header.frame_id,
            thermal_frame=thermal_message.header.frame_id,
            rgb_camera_info=json.dumps(camera_info_dict(self.rgb_info)),
            depth_camera_info=json.dumps(camera_info_dict(self.depth_info)),
            thermal_camera_info=json.dumps(camera_info_dict(self.thermal_info)),
        )
        self.views.append(
            CapturedView(
                index=index,
                path=path,
                rgb_corners=rgb_corners,
                thermal_corners=thermal_corners,
                object_points=self.points.copy(),
                rgb_size=(rgb.shape[1], rgb.shape[0]),
                thermal_size=(thermal.shape[1], thermal.shape[0]),
                rgb_frame=rgb_message.header.frame_id,
                thermal_frame=thermal_message.header.frame_id,
                relative_rotation=relative_rotation,
                relative_translation=relative_translation,
            )
        )
        header_delta = message_stamp_seconds(rgb_message) - message_stamp_seconds(
            thermal_message
        )
        print(
            f"\n{'자동 ' if automatic else ''}저장 {index:02d}: "
            f"RGB {rgb.shape[1]}x{rgb.shape[0]}, "
            f"Thermal {thermal.shape[1]}x{thermal.shape[0]} ({thermal_method}), "
            f"header Δ={header_delta:+.3f}s"
        )
        return True

    def delete_last(self) -> None:
        if not self.views:
            print("\n삭제할 캡처가 없습니다")
            return
        view = self.views.pop()
        view.path.unlink(missing_ok=True)
        print(f"\n삭제: view {view.index:03d}")

    def solve(self) -> dict:
        if len(self.views) < self.arguments.minimum_views:
            raise RuntimeError(
                f"최소 {self.arguments.minimum_views}개 자세가 필요합니다 "
                f"(현재 {len(self.views)}개)"
            )
        rgb_sizes = {view.rgb_size for view in self.views}
        thermal_sizes = {view.thermal_size for view in self.views}
        rgb_frames = {view.rgb_frame for view in self.views}
        thermal_frames = {view.thermal_frame for view in self.views}
        if len(rgb_sizes) != 1 or len(thermal_sizes) != 1:
            raise RuntimeError("캡처 도중 영상 해상도가 변경되었습니다")
        if len(rgb_frames) != 1 or len(thermal_frames) != 1:
            raise RuntimeError("캡처 도중 optical frame ID가 변경되었습니다")

        selected, rejected_views, grid_coverage = select_diverse_views(
            self.views,
            self.rgb_info,
            self.arguments.maximum_solver_views,
            self.arguments.duplicate_rgb_rms_px,
            self.arguments.duplicate_thermal_rms_px,
        )
        if len(selected) < self.arguments.minimum_views:
            raise RuntimeError(
                f"다양성·포즈 검사 후 {len(selected)}장만 남았습니다; "
                f"최소 {self.arguments.minimum_views}장이 필요합니다"
            )
        print(
            f"\n선별: 입력 {len(self.views)}장 -> 1차 {len(selected)}장, "
            f"화면 구역 {grid_coverage['occupied_cells']}/9"
        )
        (
            thermal_rms,
            thermal_k,
            thermal_d,
            stereo_rms,
            rotation,
            translation,
        ) = calibrate_view_set(selected, self.rgb_info)
        initial_errors = view_reprojection_errors(
            selected,
            self.rgb_info,
            thermal_k,
            thermal_d,
            rotation,
            translation,
        )
        error_limit = reprojection_outlier_limit(
            initial_errors, self.arguments.max_view_reprojection_rms_px
        )
        final_views = [
            view for view in selected
            if initial_errors.get(view.index, math.inf) <= error_limit
        ]
        for view in selected:
            error = initial_errors.get(view.index, math.inf)
            if error > error_limit:
                rejected_views.append({
                    "index": view.index,
                    "reason": "reprojection_outlier",
                    "rms_px": error,
                    "limit_px": error_limit,
                })
        if len(final_views) < self.arguments.minimum_views:
            raise RuntimeError(
                f"재투영 이상치 제거 후 {len(final_views)}장만 남았습니다; "
                "사진 분포와 체커보드 검출 상태를 확인하세요"
            )
        if len(final_views) != len(selected):
            print(
                f"재투영 이상치 {len(selected) - len(final_views)}장 제외 "
                f"(한계 {error_limit:.2f}px), {len(final_views)}장으로 재계산"
            )
            (
                thermal_rms,
                thermal_k,
                thermal_d,
                stereo_rms,
                rotation,
                translation,
            ) = calibrate_view_set(final_views, self.rgb_info)
        final_errors = view_reprojection_errors(
            final_views,
            self.rgb_info,
            thermal_k,
            thermal_d,
            rotation,
            translation,
        )
        thermal_size = final_views[0].thermal_size
        rgb_size = final_views[0].rgb_size
        rgb_k = camera_matrix(self.rgb_info)

        # stereoCalibrate returns X_thermal = R * X_rgb + t.  A ROS static
        # transform with RGB as parent stores the inverse pose: X_rgb from
        # X_thermal. tf2 then inverts it for lookup_transform(thermal, rgb).
        parent_rotation, parent_translation = invert_transform(
            rotation, translation
        )
        quaternion = quaternion_from_rotation(parent_rotation)
        parent_frame = next(iter(rgb_frames))
        child_frame = next(iter(thermal_frames))
        if not parent_frame or not child_frame:
            raise RuntimeError(
                "RGB 또는 thermal image의 frame_id가 비어 있습니다"
            )

        depth_frame = self.depth_info.header.frame_id
        depth_same_size = (
            int(self.depth_info.width), int(self.depth_info.height)
        ) == rgb_size
        try:
            depth_same_intrinsics = np.allclose(
                camera_matrix(self.depth_info), rgb_k, rtol=1.0e-5, atol=1.0e-5
            )
        except ValueError:
            depth_same_intrinsics = False
        depth_registration = {
            "metadata_consistent": bool(
                depth_frame == parent_frame
                and depth_same_size
                and depth_same_intrinsics
            ),
            "rgb_frame": parent_frame,
            "depth_frame": depth_frame,
            "same_resolution": depth_same_size,
            "same_intrinsics": bool(depth_same_intrinsics),
            "note": (
                "Metadata consistency is necessary but physical edge overlap "
                "must still be verified."
            ),
        }

        intrinsic_text = thermal_camera_yaml(
            thermal_size[0], thermal_size[1], thermal_k, thermal_d
        )
        extrinsic_document = {
            "parent_frame_id": parent_frame,
            "child_frame_id": child_frame,
            "translation": {
                "x": float(parent_translation[0]),
                "y": float(parent_translation[1]),
                "z": float(parent_translation[2]),
            },
            "rotation_xyzw": {
                "x": quaternion[0],
                "y": quaternion[1],
                "z": quaternion[2],
                "w": quaternion[3],
            },
            "measurement": {
                "direction": "thermal_from_rgb",
                "translation_m": translation.reshape(3).tolist(),
                "rotation_matrix": rotation.reshape(-1).tolist(),
            },
        }
        extrinsic_text = yaml.safe_dump(extrinsic_document, sort_keys=False)

        intrinsic_path = self.arguments.output_dir / "thermal_intrinsics.yaml"
        extrinsic_path = self.arguments.output_dir / "thermal_rgb_extrinsic.yaml"
        intrinsic_backup = write_with_backup(intrinsic_path, intrinsic_text)
        extrinsic_backup = write_with_backup(extrinsic_path, extrinsic_text)
        (self.session / "thermal_intrinsics.yaml").write_text(
            intrinsic_text, encoding="utf-8"
        )
        (self.session / "thermal_rgb_extrinsic.yaml").write_text(
            extrinsic_text, encoding="utf-8"
        )

        report = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "method": "diverse_grid_selection_then_reprojection_filter",
            "input_views": len(self.views),
            "initial_selected_views": len(selected),
            "views": len(final_views),
            "selected_view_indices": [view.index for view in final_views],
            "rejected_views": sorted(
                rejected_views, key=lambda item: int(item["index"])
            ),
            "grid_coverage": grid_coverage,
            "reprojection_outlier_limit_px": float(error_limit),
            "final_per_view_reprojection_rms_px": {
                str(index): float(error)
                for index, error in sorted(final_errors.items())
            },
            "board_squares": [
                self.arguments.pattern_columns + 1,
                self.arguments.pattern_rows + 1,
            ],
            "inner_corners": list(self.pattern),
            "square_size_m": self.arguments.square_size_m,
            "rgb_size": list(rgb_size),
            "thermal_size": list(thermal_size),
            "rgb_frame": parent_frame,
            "thermal_frame": child_frame,
            "thermal_rms_px": float(thermal_rms),
            "stereo_rms_px": float(stereo_rms),
            "thermal_k": thermal_k.reshape(-1).tolist(),
            "thermal_d": thermal_d.reshape(-1).tolist(),
            "thermal_from_rgb_translation_m": translation.reshape(3).tolist(),
            "thermal_from_rgb_rotation_matrix": rotation.reshape(-1).tolist(),
            "hp60c_depth_registration": depth_registration,
            "intrinsic_path": str(intrinsic_path),
            "extrinsic_path": str(extrinsic_path),
            "backups": [
                str(path) for path in (intrinsic_backup, extrinsic_backup) if path
            ],
        }
        report_text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
        (self.session / "report.json").write_text(report_text, encoding="utf-8")
        write_with_backup(self.arguments.output_dir / "latest_report.json", report_text)
        return report


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--rgb-topic",
        default="/ascamera_hp60c/camera_publisher/rgb0/image",
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
    result.add_argument(
        "--thermal-topic", default="/thermal_camera/image_raw"
    )
    result.add_argument(
        "--thermal-info-topic", default="/thermal_camera/camera_info"
    )
    result.add_argument("--pattern-columns", type=int, default=PATTERN_COLUMNS)
    result.add_argument("--pattern-rows", type=int, default=PATTERN_ROWS)
    result.add_argument("--square-size-m", type=float, default=SQUARE_SIZE_M)
    result.add_argument("--minimum-views", type=int, default=15)
    result.add_argument(
        "--maximum-solver-views",
        type=int,
        default=40,
        help="maximum diverse views passed to the expensive calibration solve",
    )
    result.add_argument(
        "--max-view-reprojection-rms-px", type=float, default=4.0
    )
    result.add_argument(
        "--auto-capture-interval-sec",
        type=float,
        default=0.0,
        help="seconds between automatic capture attempts; 0 disables auto capture",
    )
    result.add_argument("--stable-duration-sec", type=float, default=0.8)
    result.add_argument("--stability-rgb-rms-px", type=float, default=1.5)
    result.add_argument("--stability-thermal-rms-px", type=float, default=0.75)
    result.add_argument("--duplicate-rgb-rms-px", type=float, default=6.0)
    result.add_argument("--duplicate-thermal-rms-px", type=float, default=2.0)
    result.add_argument("--max-receipt-offset-sec", type=float, default=0.20)
    result.add_argument(
        "--min-thermal-board-area-px2", type=float, default=400.0
    )
    result.add_argument("--thermal-border-margin-px", type=float, default=2.0)
    result.add_argument("--output-dir", type=Path, default=DEFAULT_ROOT)
    result.add_argument(
        "--no-gui",
        action="store_true",
        help="disable the RGB/depth/thermal OpenCV preview window",
    )
    return result


def main(argv: list[str] | None = None) -> int:
    arguments = parser().parse_args(argv)
    arguments.output_dir = arguments.output_dir.expanduser().resolve()
    if arguments.pattern_columns < 2 or arguments.pattern_rows < 2:
        raise SystemExit("pattern dimensions are inner-corner counts and must be >= 2")
    if arguments.square_size_m <= 0.0:
        raise SystemExit("square-size-m must be positive")
    positive_names = (
        "stable_duration_sec",
        "stability_rgb_rms_px",
        "stability_thermal_rms_px",
        "duplicate_rgb_rms_px",
        "duplicate_thermal_rms_px",
        "max_receipt_offset_sec",
        "min_thermal_board_area_px2",
        "max_view_reprojection_rms_px",
    )
    if arguments.auto_capture_interval_sec < 0.0:
        raise SystemExit("auto-capture-interval-sec must be >= 0")
    if any(getattr(arguments, name) <= 0.0 for name in positive_names):
        raise SystemExit("automatic capture quality thresholds must be positive")
    if arguments.thermal_border_margin_px < 0.0:
        raise SystemExit("thermal-border-margin-px must be >= 0")
    if arguments.minimum_views < 3:
        raise SystemExit("minimum-views must be >= 3")
    if arguments.maximum_solver_views < arguments.minimum_views:
        raise SystemExit("maximum-solver-views must be >= minimum-views")

    rclpy.init()
    node = PhysicalThermalCalibration(arguments)
    print("\n실물 RGB-D ↔ 열화상 캘리브레이션")
    print(
        f"체커보드: {arguments.pattern_columns + 1}x"
        f"{arguments.pattern_rows + 1}칸, 내부 코너 "
        f"{arguments.pattern_columns}x{arguments.pattern_rows}, "
        f"칸 {arguments.square_size_m * 1000:.1f} mm"
    )
    print(f"세션 저장: {node.session}")
    if arguments.auto_capture_interval_sec > 0.0:
        print(
            f"자동 저장: {arguments.auto_capture_interval_sec:.1f}초 간격, "
            f"{arguments.stable_duration_sec:.1f}초 정지 후 품질 통과 자세만 저장"
        )
    print("SPACE 수동 저장 | D 마지막 삭제 | S 계산·저장 | Q 종료")
    if not arguments.no_gui:
        print("미리보기: RGB | Depth | Thermal (GUI 창에서도 키 입력 가능)")

    last_status = 0.0
    try:
        with TerminalKeys() as keys:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if now - last_status >= 1.0:
                    ready, missing = node.readiness()
                    status = "수신 정상" if ready else f"대기: {','.join(missing)}"
                    print(
                        f"\r[{status}] 저장 {len(node.views):02d}개 ",
                        end="",
                        flush=True,
                    )
                    last_status = now
                terminal_key = keys.read()
                gui_key = node.update_preview()
                node.maybe_auto_capture()
                key = terminal_key or gui_key
                if key == " ":
                    node.capture()
                elif key and key.lower() == "d":
                    node.delete_last()
                elif key and key.lower() == "s":
                    try:
                        report = node.solve()
                    except Exception as error:  # noqa: BLE001
                        print(f"\n계산 실패: {error}")
                        continue
                    print("\n캘리브레이션 저장 완료")
                    print(
                        f"  selected views: {report['views']}/"
                        f"{report['input_views']}"
                    )
                    print(f"  thermal RMS: {report['thermal_rms_px']:.3f} px")
                    print(f"  stereo RMS:  {report['stereo_rms_px']:.3f} px")
                    registration = report["hp60c_depth_registration"]
                    print(
                        "  HP60C registration metadata: "
                        + ("일치" if registration["metadata_consistent"] else "불일치")
                    )
                    print(f"  intrinsic: {report['intrinsic_path']}")
                    print(f"  extrinsic: {report['extrinsic_path']}")
                    return 0
                elif key and key.lower() == "q":
                    print("\n계산하지 않고 종료합니다. 캡처 원본은 보존됩니다.")
                    return 0
    except KeyboardInterrupt:
        print("\n중단했습니다. 캡처 원본은 보존됩니다.")
        return 130
    finally:
        node.close_preview()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
