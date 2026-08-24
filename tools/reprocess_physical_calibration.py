#!/usr/bin/env python3
"""Robustly reprocess a saved physical RGB/thermal calibration session."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml


DEFAULT_ROOT = Path("~/.local/share/hazard_guard/calibration").expanduser()


def rotation_distance(first, second):
    cosine = np.clip((np.trace(first.T @ second) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(cosine))


def rotation_to_rpy(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float64)


def quaternion_from_rotation(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2
            values = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(
                1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2
            values = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(
                1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2
            values = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    return (values / np.linalg.norm(values)).tolist()


def camera_matrix(info):
    return np.asarray(info["k"], dtype=np.float64).reshape(3, 3)


def distortion(info):
    values = np.asarray(info["d"], dtype=np.float64)
    return values.reshape(-1, 1) if values.size else np.zeros((5, 1))


def load_views(session):
    views = []
    for path in sorted(session.glob("view_*.npz")):
        data = np.load(path)
        views.append({
            "index": int(path.stem.split("_")[-1]),
            "path": path,
            "object": data["object_points"].astype(np.float32),
            "rgb": data["rgb_corners"].astype(np.float32),
            "thermal": data["thermal_corners"].astype(np.float32),
            "rgb_size": (int(data["rgb"].shape[1]), int(data["rgb"].shape[0])),
            "thermal_size": (
                int(data["thermal"].shape[1]), int(data["thermal"].shape[0])
            ),
            "rgb_frame": str(data["rgb_frame"]),
            "thermal_frame": str(data["thermal_frame"]),
            "rgb_info": json.loads(str(data["rgb_camera_info"])),
            "depth_info": json.loads(str(data["depth_camera_info"])),
        })
    if not views:
        raise RuntimeError(f"no view_*.npz files in {session}")
    return views


def remove_duplicates(views, threshold):
    unique = []
    rejected = []
    for view in views:
        duplicate = None
        for previous in unique:
            rgb_error = float(np.sqrt(np.mean((view["rgb"] - previous["rgb"]) ** 2)))
            thermal_error = float(
                np.sqrt(np.mean((view["thermal"] - previous["thermal"]) ** 2))
            )
            if rgb_error < threshold and thermal_error < threshold:
                duplicate = previous["index"]
                break
        if duplicate is None:
            unique.append(view)
        else:
            rejected.append({
                "index": view["index"],
                "reason": f"duplicate_of_view_{duplicate:03d}",
            })
    return unique, rejected


def calibrate_thermal(views):
    objects = [v["object"].reshape(-1, 1, 3) for v in views]
    points = [v["thermal"].reshape(-1, 1, 2) for v in views]
    size = views[0]["thermal_size"]
    guess = cv2.initCameraMatrix2D(objects, points, size)
    return cv2.calibrateCamera(
        objects, points, size, guess, None,
        flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3,
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            200, 1.0e-10,
        ),
    )[:3]


def relative_pose(view, thermal_k, thermal_d):
    rgb_k = camera_matrix(view["rgb_info"])
    rgb_d = distortion(view["rgb_info"])
    _, rgb_rvec, rgb_tvec = cv2.solvePnP(
        view["object"], view["rgb"], rgb_k, rgb_d
    )
    _, thermal_rvec, thermal_tvec = cv2.solvePnP(
        view["object"], view["thermal"], thermal_k, thermal_d
    )
    rgb_rotation = cv2.Rodrigues(rgb_rvec)[0]
    thermal_rotation = cv2.Rodrigues(thermal_rvec)[0]
    rotation = thermal_rotation @ rgb_rotation.T
    translation = (
        thermal_tvec.reshape(3) - rotation @ rgb_tvec.reshape(3)
    )
    return rotation, translation


def reject_pose_outliers(views, thermal_k, thermal_d):
    poses = [relative_pose(view, thermal_k, thermal_d) for view in views]
    translations = np.asarray([pose[1] for pose in poses])
    translation_centre = np.median(translations, axis=0)
    medoid = min(
        range(len(poses)),
        key=lambda index: sum(
            rotation_distance(poses[index][0], other[0]) for other in poses
        ),
    )
    rotation_centre = poses[medoid][0]
    translation_errors = np.linalg.norm(
        translations - translation_centre, axis=1
    )
    rotation_errors = np.asarray([
        rotation_distance(rotation_centre, pose[0]) for pose in poses
    ])

    def threshold(errors, floor):
        median = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median)))
        return max(floor, median + 4.0 * 1.4826 * mad)

    translation_limit = threshold(translation_errors, 0.08)
    rotation_limit = threshold(rotation_errors, math.radians(10.0))
    accepted = []
    rejected = []
    diagnostics = []
    for view, translation_error, rotation_error in zip(
        views, translation_errors, rotation_errors
    ):
        diagnostic = {
            "index": view["index"],
            "translation_error_m": float(translation_error),
            "rotation_error_deg": math.degrees(float(rotation_error)),
        }
        diagnostics.append(diagnostic)
        if translation_error > translation_limit or rotation_error > rotation_limit:
            rejected.append({**diagnostic, "reason": "relative_pose_outlier"})
        else:
            accepted.append(view)
    limits = {
        "translation_m": translation_limit,
        "rotation_deg": math.degrees(rotation_limit),
    }
    return accepted, rejected, diagnostics, limits


def solve_stereo(views, thermal_k, thermal_d):
    objects = [v["object"].reshape(-1, 1, 3) for v in views]
    rgb_points = [v["rgb"].reshape(-1, 1, 2) for v in views]
    thermal_points = [v["thermal"].reshape(-1, 1, 2) for v in views]
    rgb_k = camera_matrix(views[0]["rgb_info"])
    rgb_d = distortion(views[0]["rgb_info"])
    result = cv2.stereoCalibrate(
        objects, rgb_points, thermal_points,
        rgb_k.copy(), rgb_d.copy(), thermal_k.copy(), thermal_d.copy(),
        views[0]["rgb_size"], flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            300, 1.0e-10,
        ),
    )
    return result[0], result[5], result[6].reshape(3)


def thermal_yaml(size, matrix, coefficients):
    distortion_values = coefficients.reshape(-1).tolist()
    projection = [
        float(matrix[0, 0]), 0.0, float(matrix[0, 2]), 0.0,
        0.0, float(matrix[1, 1]), float(matrix[1, 2]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    document = {
        "image_width": size[0], "image_height": size[1],
        "camera_name": "thermal_camera",
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix.reshape(-1).tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1, "cols": len(distortion_values), "data": distortion_values,
        },
        "rectification_matrix": {
            "rows": 3, "cols": 3, "data": np.eye(3).reshape(-1).tolist(),
        },
        "projection_matrix": {"rows": 3, "cols": 4, "data": projection},
    }
    return yaml.safe_dump(document, sort_keys=False)


def write_backup(path, text):
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


def latest_session(root):
    sessions = sorted((root / "sessions").glob("physical-*/"))
    if not sessions:
        raise RuntimeError(f"no physical sessions under {root / 'sessions'}")
    return sessions[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--duplicate-threshold-px", type=float, default=0.5)
    parser.add_argument("--max-stereo-rms-px", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args()
    root = arguments.root.expanduser().resolve()
    session = (
        arguments.session.expanduser().resolve()
        if arguments.session else latest_session(root)
    )
    views = load_views(session)
    unique, duplicate_rejections = remove_duplicates(
        views, arguments.duplicate_threshold_px
    )
    _, initial_k, initial_d = calibrate_thermal(unique)
    accepted, pose_rejections, diagnostics, limits = reject_pose_outliers(
        unique, initial_k, initial_d
    )
    if len(accepted) < 10:
        raise RuntimeError(f"only {len(accepted)} valid views remain")
    thermal_rms, thermal_k, thermal_d = calibrate_thermal(accepted)
    stereo_rms, rotation, translation = solve_stereo(
        accepted, thermal_k, thermal_d
    )
    if stereo_rms > arguments.max_stereo_rms_px:
        raise RuntimeError(
            f"stereo RMS {stereo_rms:.3f}px exceeds "
            f"{arguments.max_stereo_rms_px:.3f}px; files were not replaced"
        )

    # OpenCV: X_thermal = R * X_rgb + t. ROS parent RGB -> child thermal
    # stores the inverse pose (thermal origin and axes expressed in RGB).
    parent_rotation = rotation.T
    parent_translation = -parent_rotation @ translation
    parent_rpy = rotation_to_rpy(parent_rotation)
    measurement_rpy = rotation_to_rpy(rotation)
    quaternion = quaternion_from_rotation(parent_rotation)
    intrinsic_text = thermal_yaml(
        accepted[0]["thermal_size"], thermal_k, thermal_d
    )
    extrinsic = {
        "parent_frame_id": accepted[0]["rgb_frame"],
        "child_frame_id": accepted[0]["thermal_frame"],
        "translation": dict(zip(("x", "y", "z"), parent_translation.tolist())),
        "rotation_xyzw": dict(zip(("x", "y", "z", "w"), quaternion)),
        "rpy_rad": dict(zip(("roll", "pitch", "yaw"), parent_rpy.tolist())),
        "rpy_deg": dict(zip(
            ("roll", "pitch", "yaw"), np.degrees(parent_rpy).tolist()
        )),
        "measurement": {
            "direction": "thermal_from_rgb",
            "translation_m": translation.tolist(),
            "rotation_matrix": rotation.reshape(-1).tolist(),
            "rpy_rad": measurement_rpy.tolist(),
            "rpy_deg": np.degrees(measurement_rpy).tolist(),
        },
    }
    extrinsic_text = yaml.safe_dump(extrinsic, sort_keys=False)
    old_report = {}
    old_report_path = root / "latest_report.json"
    if old_report_path.is_file():
        old_report = json.loads(old_report_path.read_text(encoding="utf-8"))
    report = {
        **old_report,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "method": "robust_saved_session_reprocess",
        "source_session": str(session),
        "input_views": len(views),
        "views": len(accepted),
        "accepted_view_indices": [view["index"] for view in accepted],
        "rejected_views": duplicate_rejections + pose_rejections,
        "rejection_limits": limits,
        "per_view_pose_diagnostics": diagnostics,
        "thermal_rms_px": float(thermal_rms),
        "stereo_rms_px": float(stereo_rms),
        "thermal_k": thermal_k.reshape(-1).tolist(),
        "thermal_d": thermal_d.reshape(-1).tolist(),
        "thermal_from_rgb_translation_m": translation.tolist(),
        "thermal_from_rgb_rotation_matrix": rotation.reshape(-1).tolist(),
        "thermal_from_rgb_rpy_rad": measurement_rpy.tolist(),
        "thermal_from_rgb_rpy_deg": np.degrees(measurement_rpy).tolist(),
        "ros_parent_rgb_child_thermal_xyz_m": parent_translation.tolist(),
        "ros_parent_rgb_child_thermal_rpy_rad": parent_rpy.tolist(),
        "ros_parent_rgb_child_thermal_rpy_deg": np.degrees(parent_rpy).tolist(),
    }
    report_text = json.dumps(report, indent=2, ensure_ascii=False) + "\n"

    print(f"session: {session}")
    print(f"accepted: {[view['index'] for view in accepted]}")
    print(f"rejected: {duplicate_rejections + pose_rejections}")
    print(f"thermal RMS: {thermal_rms:.3f} px")
    print(f"stereo RMS:  {stereo_rms:.3f} px")
    print(f"ROS xyz m:   {np.round(parent_translation, 6).tolist()}")
    print(f"ROS rpy rad: {np.round(parent_rpy, 6).tolist()}")
    print(f"ROS rpy deg: {np.round(np.degrees(parent_rpy), 3).tolist()}")
    if arguments.dry_run:
        print("dry-run: no files changed")
        return

    backups = []
    for path, text in (
        (root / "thermal_intrinsics.yaml", intrinsic_text),
        (root / "thermal_rgb_extrinsic.yaml", extrinsic_text),
        (root / "latest_report.json", report_text),
    ):
        backup = write_backup(path, text)
        if backup:
            backups.append(str(backup))
    (session / "thermal_intrinsics.robust.yaml").write_text(
        intrinsic_text, encoding="utf-8"
    )
    (session / "thermal_rgb_extrinsic.robust.yaml").write_text(
        extrinsic_text, encoding="utf-8"
    )
    (session / "report.robust.json").write_text(
        report_text, encoding="utf-8"
    )
    print("backups:")
    for backup in backups:
        print(f"  {backup}")
    print("robust calibration installed; restart the thermal ROS launch to load it")


if __name__ == "__main__":
    main()
