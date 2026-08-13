"""Find the board in both cameras and pair up the points that are the same.

The RGB side is ordinary: a 640 x 480 frame of a 50 mm chequerboard gives
corners that OpenCV's sector-based detector finds with sub-pixel accuracy and
no help.

The thermal side is the paper's actual problem. At 160 x 120 a 100 mm square
spans about 16 px at 0.9 m, which is enough for the same detector to work - but
only after the frame is turned into something a detector recognises. The stream
is 16-bit Kelvin, and a scene containing a 45 C board and a 15 C room maps to a
narrow band of the 16-bit range; stretched naively on min and max, one hot pipe
in the corner flattens the board to two adjacent grey levels. So the paper's
first step is percentile normalisation, and it is the first step here.

Detection is tried in three escalating forms, cheapest first, because the
escalations cost accuracy as well as time: upsampling invents no information
and CLAHE amplifies noise along with contrast. Which one fired is recorded per
frame, so the resolution sweep can report how far down the ladder each
resolution had to go - that number is the paper's contribution measured on our
own data.

The two grids have different densities, so most RGB corners have no thermal
partner. The ones that do are given by the board's construction rule,
rgb = 2 * tir + 1, and pairing is a lookup rather than a match.
"""
from __future__ import annotations

import cv2
import numpy as np

# Percentiles for the thermal stretch, from the paper. Not min/max: a single
# hot or cold outlier anywhere in frame would otherwise set the whole scale.
LOW_PERCENTILE, HIGH_PERCENTILE = 1.0, 99.0

# How much to enlarge before the fallback attempts. The paper uses 4x bicubic.
UPSAMPLE = 4


def normalise_thermal(thermal: np.ndarray) -> np.ndarray:
    """16-bit Kelvin x 100 to an 8-bit image a corner detector will accept."""
    celsius = thermal.astype(np.float32) / 100.0 - 273.15
    low, high = np.percentile(celsius, [LOW_PERCENTILE, HIGH_PERCENTILE])
    if high - low < 1e-3:
        return np.zeros(celsius.shape, np.uint8)
    return np.clip((celsius - low) / (high - low) * 255.0, 0, 255).astype(np.uint8)


def _sb(gray: np.ndarray, pattern: tuple):
    found, corners = cv2.findChessboardCornersSB(
        gray, pattern, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return corners.reshape(-1, 2) if found else None


def _classic(gray: np.ndarray, pattern: tuple):
    found, corners = cv2.findChessboardCorners(
        gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4))
    return corners.reshape(-1, 2)


def detect_rgb(rgb: np.ndarray, pattern: tuple):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY) if rgb.ndim == 3 else rgb
    corners = _sb(gray, pattern)
    return (corners, "sb") if corners is not None else (None, "none")


def detect_thermal(thermal: np.ndarray, pattern: tuple):
    """Escalating attempts; returns corners in original image coordinates."""
    gray = normalise_thermal(thermal)

    corners = _sb(gray, pattern)
    if corners is not None:
        return corners, "sb"

    # Enlarging does not add information, but the detectors have kernel sizes
    # and minimum-feature assumptions written for ordinary images, and at
    # 16 px a square is near those floors.
    big = cv2.resize(gray, None, fx=UPSAMPLE, fy=UPSAMPLE,
                     interpolation=cv2.INTER_CUBIC)
    corners = _sb(big, pattern)
    if corners is not None:
        return corners / UPSAMPLE, "sb-upsampled"

    # CLAHE last: it lifts local contrast where the board is dim against the
    # room, at the cost of amplifying whatever noise is there too.
    equalised = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(big)
    corners = _sb(equalised, pattern)
    if corners is not None:
        return corners / UPSAMPLE, "sb-clahe"

    corners = _classic(equalised, pattern)
    if corners is not None:
        return corners / UPSAMPLE, "classic-clahe"
    return None, "none"


def same_orientation(rgb_corners: np.ndarray, tir_corners: np.ndarray) -> bool:
    """Do the two detections run the same way round the board?

    A chequerboard with no markers is ambiguous under a 180 degree rotation, so
    a detector may return either end first. The ambiguity itself is harmless -
    a flipped board is a board pose rotated about its own normal, and the
    optimiser solves for that pose anyway - but only if both cameras flip
    together. They sit 68 mm apart and see the same view, so a disagreement is
    the detector's doing, not the geometry's, and pairing point i with point i
    across a disagreement would match opposite corners of the board.
    """
    first = rgb_corners[-1] - rgb_corners[0]
    second = tir_corners[-1] - tir_corners[0]
    return float(np.dot(first, second)) > 0


def board_points(spec: dict) -> np.ndarray:
    """Every RGB interior corner in board coordinates, metres, z = 0.

    Centred on the board so the numbers stay small and symmetric; the board
    pose is free per view, so the origin's placement changes nothing.
    """
    columns, rows = spec["rgb_corners"]
    square = spec["square_m"]
    points = np.zeros((rows * columns, 3), np.float64)
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, 0] = (grid[:, 0] + 1) * square - spec["width_m"] / 2
    points[:, 1] = (grid[:, 1] + 1) * square - spec["height_m"] / 2
    return points


def matched_indices(spec: dict):
    """(rgb index, tir index) pairs, from the board's construction rule."""
    rgb_columns = spec["rgb_corners"][0]
    tir_columns, tir_rows = spec["tir_corners"]
    factor = spec["coarse_factor"]
    rgb_index, tir_index = [], []
    for n in range(tir_rows):
        for m in range(tir_columns):
            rgb_index.append((factor * n + 1) * rgb_columns + (factor * m + 1))
            tir_index.append(n * tir_columns + m)
    return np.array(rgb_index), np.array(tir_index)


def detect_pair(rgb: np.ndarray, thermal: np.ndarray, spec: dict):
    """Both detections plus the matched subset, or a reason it was rejected."""
    rgb_pattern = tuple(spec["rgb_corners"])
    tir_pattern = tuple(spec["tir_corners"])

    rgb_corners, rgb_how = detect_rgb(rgb, rgb_pattern)
    if rgb_corners is None:
        return None, {"reason": "rgb", "rgb_how": rgb_how, "tir_how": "-"}
    tir_corners, tir_how = detect_thermal(thermal, tir_pattern)
    if tir_corners is None:
        return None, {"reason": "thermal", "rgb_how": rgb_how, "tir_how": tir_how}

    flipped = not same_orientation(rgb_corners, tir_corners)
    if flipped:
        tir_corners = tir_corners[::-1].copy()

    rgb_index, tir_index = matched_indices(spec)
    return {
        "rgb_corners": rgb_corners,
        "tir_corners": tir_corners,
        "rgb_matched": rgb_corners[rgb_index],
        "tir_matched": tir_corners[tir_index],
        "object_points": board_points(spec),
        "object_matched": board_points(spec)[rgb_index],
    }, {"reason": "ok", "rgb_how": rgb_how, "tir_how": tir_how, "flipped": flipped}
