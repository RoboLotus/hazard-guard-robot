"""HazardGuard person detection package."""

from .detection import PersonDetection, detections_from_ultralytics
from .distance import DistanceEstimate, estimate_bbox_distance

__all__ = [
    "DistanceEstimate",
    "PersonDetection",
    "detections_from_ultralytics",
    "estimate_bbox_distance",
]
