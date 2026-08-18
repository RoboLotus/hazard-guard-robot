from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera dimensions must be positive")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")

    @classmethod
    def from_horizontal_fov(
        cls, width: int, height: int, horizontal_fov_deg: float
    ) -> "CameraIntrinsics":
        if width <= 0 or height <= 0:
            raise ValueError("camera dimensions must be positive")
        if not 0.0 < horizontal_fov_deg < 180.0:
            raise ValueError("horizontal FOV must be between 0 and 180 degrees")
        focal_length = width / (
            2.0 * math.tan(math.radians(horizontal_fov_deg) * 0.5)
        )
        return cls(
            width=width,
            height=height,
            fx=focal_length,
            fy=focal_length,
            cx=(width - 1) * 0.5,
            cy=(height - 1) * 0.5,
        )


@dataclass(frozen=True)
class RigidTransform:
    """Translation and xyzw quaternion from a source to a target frame."""

    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    def apply(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        norm = math.sqrt(
            self.qx * self.qx
            + self.qy * self.qy
            + self.qz * self.qz
            + self.qw * self.qw
        )
        if norm <= 1.0e-12:
            raise ValueError("transform quaternion must not be zero")
        qx = self.qx / norm
        qy = self.qy / norm
        qz = self.qz / norm
        qw = self.qw / norm

        rx = (
            (1.0 - 2.0 * (qy * qy + qz * qz)) * x
            + 2.0 * (qx * qy - qz * qw) * y
            + 2.0 * (qx * qz + qy * qw) * z
        )
        ry = (
            2.0 * (qx * qy + qz * qw) * x
            + (1.0 - 2.0 * (qx * qx + qz * qz)) * y
            + 2.0 * (qy * qz - qx * qw) * z
        )
        rz = (
            2.0 * (qx * qz - qy * qw) * x
            + 2.0 * (qy * qz + qx * qw) * y
            + (1.0 - 2.0 * (qx * qx + qy * qy)) * z
        )
        return rx + self.tx, ry + self.ty, rz + self.tz


@dataclass(frozen=True)
class ThermalPoint:
    x: float
    y: float
    z: float
    temperature_c: float
    confidence: float
    pixel_u: float = -1.0
    pixel_v: float = -1.0


def fuse_depth_and_thermal(
    depth_m: Sequence[float],
    depth_camera: CameraIntrinsics,
    temperature_c: Sequence[float],
    thermal_camera: CameraIntrinsics,
    thermal_from_depth: RigidTransform,
    *,
    stride: int = 4,
    min_depth_m: float = 0.2,
    max_depth_m: float = 4.0,
    min_temperature_c: float = -100.0,
    max_temperature_c: float = 1000.0,
) -> list[ThermalPoint]:
    """Project depth pixels into the thermal camera and attach temperature."""

    depth_camera.validate()
    thermal_camera.validate()
    if len(depth_m) < depth_camera.width * depth_camera.height:
        raise ValueError("depth image buffer is too short")
    if len(temperature_c) < thermal_camera.width * thermal_camera.height:
        raise ValueError("thermal image buffer is too short")
    if stride <= 0:
        raise ValueError("stride must be positive")

    points: list[ThermalPoint] = []
    for v in range(0, depth_camera.height, stride):
        row = v * depth_camera.width
        for u in range(0, depth_camera.width, stride):
            depth = float(depth_m[row + u])
            if not math.isfinite(depth) or not min_depth_m <= depth <= max_depth_m:
                continue
            x_depth = (u - depth_camera.cx) * depth / depth_camera.fx
            y_depth = (v - depth_camera.cy) * depth / depth_camera.fy
            x_thermal, y_thermal, z_thermal = thermal_from_depth.apply(
                x_depth, y_depth, depth
            )
            if z_thermal <= 0.0:
                continue
            thermal_u = int(
                round(thermal_camera.fx * x_thermal / z_thermal + thermal_camera.cx)
            )
            thermal_v = int(
                round(thermal_camera.fy * y_thermal / z_thermal + thermal_camera.cy)
            )
            if not (
                0 <= thermal_u < thermal_camera.width
                and 0 <= thermal_v < thermal_camera.height
            ):
                continue
            temperature = float(
                temperature_c[thermal_v * thermal_camera.width + thermal_u]
            )
            if (
                not math.isfinite(temperature)
                or temperature < min_temperature_c
                or temperature > max_temperature_c
            ):
                continue

            nx = abs((thermal_u - thermal_camera.cx) / max(thermal_camera.cx, 1.0))
            ny = abs((thermal_v - thermal_camera.cy) / max(thermal_camera.cy, 1.0))
            edge_factor = max(0.25, 1.0 - 0.35 * max(nx, ny))
            range_factor = max(
                0.25,
                1.0
                - 0.35
                * (depth - min_depth_m)
                / max(max_depth_m - min_depth_m, 1.0e-6),
            )
            points.append(
                ThermalPoint(
                    x=x_thermal,
                    y=y_thermal,
                    z=z_thermal,
                    temperature_c=temperature,
                    confidence=min(1.0, edge_factor * range_factor),
                    pixel_u=float(thermal_u),
                    pixel_v=float(thermal_v),
                )
            )
    return points
