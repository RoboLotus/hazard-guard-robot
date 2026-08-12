"""Thermal perception shared by simulation and the physical robot."""

from .projection import CameraIntrinsics, RigidTransform, ThermalPoint
from .voxel import AnalysisConfig, AxisAlignedRoi, analyze_points, load_config

__all__ = [
    "AnalysisConfig",
    "AxisAlignedRoi",
    "CameraIntrinsics",
    "RigidTransform",
    "ThermalPoint",
    "analyze_points",
    "load_config",
]
