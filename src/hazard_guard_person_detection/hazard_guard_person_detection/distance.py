"""Robust RGB-D distance estimation helpers."""

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class DistanceEstimate:
    distance_m: float
    valid: bool
    valid_sample_count: int


def estimate_bbox_distance(
    depth_m: np.ndarray,
    bbox_xyxy: Sequence[float],
    *,
    central_roi_ratio: float = 0.5,
    minimum_distance_m: float = 0.15,
    maximum_distance_m: float = 8.0,
    minimum_valid_samples: int = 9,
) -> DistanceEstimate:
    """Estimate person distance using the median of a central bbox region.

    ``depth_m`` must already be expressed in metres.  Zero, NaN, infinite, and
    out-of-range samples are discarded.  The median resists isolated depth
    holes and background pixels substantially better than the center pixel.
    """

    if depth_m.ndim != 2 or len(bbox_xyxy) < 4:
        return DistanceEstimate(0.0, False, 0)
    if not 0.0 < central_roi_ratio <= 1.0:
        raise ValueError("central_roi_ratio must be in the interval (0, 1]")
    if minimum_distance_m < 0.0 or maximum_distance_m <= minimum_distance_m:
        raise ValueError("distance bounds are invalid")

    height, width = depth_m.shape
    x_min, y_min, x_max, y_max = map(float, bbox_xyxy[:4])
    x_min = max(0.0, min(float(width), x_min))
    x_max = max(0.0, min(float(width), x_max))
    y_min = max(0.0, min(float(height), y_min))
    y_max = max(0.0, min(float(height), y_max))
    if x_max <= x_min or y_max <= y_min:
        return DistanceEstimate(0.0, False, 0)

    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    half_width = (x_max - x_min) * central_roi_ratio / 2.0
    half_height = (y_max - y_min) * central_roi_ratio / 2.0
    roi_x_min = max(0, int(np.floor(center_x - half_width)))
    roi_x_max = min(width, int(np.ceil(center_x + half_width)))
    roi_y_min = max(0, int(np.floor(center_y - half_height)))
    roi_y_max = min(height, int(np.ceil(center_y + half_height)))

    roi = np.asarray(depth_m[roi_y_min:roi_y_max, roi_x_min:roi_x_max], dtype=np.float32)
    valid = roi[
        np.isfinite(roi)
        & (roi > 0.0)
        & (roi >= minimum_distance_m)
        & (roi <= maximum_distance_m)
    ]
    count = int(valid.size)
    if count < minimum_valid_samples:
        return DistanceEstimate(0.0, False, count)
    return DistanceEstimate(float(np.median(valid)), True, count)


def depth_image_to_metres(depth: np.ndarray, encoding: Optional[str]) -> np.ndarray:
    """Normalize common ROS depth encodings to metres without mutating input."""

    normalized_encoding = (encoding or "").upper()
    converted = np.asarray(depth, dtype=np.float32)
    if normalized_encoding in {"16UC1", "MONO16"}:
        converted = converted * 0.001
    return converted
