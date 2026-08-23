"""Pure conversion helpers for person detector output.

This module deliberately does not import Ultralytics, torch, OpenCV, or ROS.  It
can therefore be unit-tested on development machines that do not have a GPU or
the inference runtime installed.
"""

from dataclasses import dataclass
import math
from typing import Any, Iterable, List


@dataclass(frozen=True)
class PersonDetection:
    """A single person bounding box in image pixel coordinates."""

    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) / 2.0

    @property
    def size_x(self) -> float:
        return max(0.0, self.x_max - self.x_min)

    @property
    def size_y(self) -> float:
        return max(0.0, self.y_max - self.y_min)


def _as_list(value: Any) -> List[Any]:
    """Convert torch/numpy/list-like values to a Python list.

    The outer dimension is deliberately preserved.  Ultralytics represents a
    single bounding box as ``[[x1, y1, x2, y2]]``; flattening that value here
    would make the one-person case disappear when rows are zipped below.
    """

    if value is None:
        return []
    current = value
    for attribute in ("detach", "cpu"):
        method = getattr(current, attribute, None)
        if callable(method):
            current = method()
    tolist = getattr(current, "tolist", None)
    if callable(tolist):
        current = tolist()
    if isinstance(current, (str, bytes)):
        return [current]
    if not isinstance(current, Iterable):
        return [current]
    return list(current)


def detections_from_ultralytics(
    result: Any,
    *,
    person_class_id: int = 0,
    minimum_confidence: float = 0.0,
) -> List[PersonDetection]:
    """Convert one Ultralytics result object into filtered person detections.

    Only the small ``boxes.cls``, ``boxes.conf`` and ``boxes.xyxy`` protocol is
    required.  Fake objects implementing that protocol can be used in tests.
    Malformed rows are skipped instead of taking the ROS node down.
    """

    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []

    class_ids = _as_list(getattr(boxes, "cls", None))
    confidences = _as_list(getattr(boxes, "conf", None))
    coordinates = _as_list(getattr(boxes, "xyxy", None))

    detections: List[PersonDetection] = []
    for class_id, confidence, row in zip(class_ids, confidences, coordinates):
        try:
            confidence_value = float(confidence)
            coordinate_values = _as_list(row)
            if int(class_id) != person_class_id:
                continue
            if (
                not math.isfinite(confidence_value)
                or confidence_value < minimum_confidence
                or len(coordinate_values) < 4
            ):
                continue
            x_min, y_min, x_max, y_max = map(float, coordinate_values[:4])
            if not all(math.isfinite(value) for value in (x_min, y_min, x_max, y_max)):
                continue
            if x_max <= x_min or y_max <= y_min:
                continue
        except (TypeError, ValueError, OverflowError):
            continue
        detections.append(
            PersonDetection(
                confidence=confidence_value,
                x_min=x_min,
                y_min=y_min,
                x_max=x_max,
                y_max=y_max,
            )
        )
    return detections
