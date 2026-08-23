from __future__ import annotations

import math
import struct
from typing import Iterable, Iterator

import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

from .projection import ThermalPoint


THERMAL_POINT_FIELDS = (
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="temperature_c", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="confidence", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="pixel_u", offset=20, datatype=PointField.FLOAT32, count=1),
    PointField(name="pixel_v", offset=24, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=28, datatype=PointField.UINT32, count=1),
)
THERMAL_POINT_STEP = 32


def temperature_rgb(temperature_c: float, low_c: float, high_c: float) -> int:
    """Return an opaque browser/PCL-compatible 0x00RRGGBB heat-map colour."""
    span = max(float(high_c) - float(low_c), 1.0e-6)
    value = min(1.0, max(0.0, (float(temperature_c) - low_c) / span))
    # Compact blue -> cyan -> yellow -> red ramp. The fixed temperature window
    # keeps the same physical temperature the same colour between frames.
    red = min(1.0, max(0.0, 1.5 - abs(4.0 * value - 3.0)))
    green = min(1.0, max(0.0, 1.5 - abs(4.0 * value - 2.0)))
    blue = min(1.0, max(0.0, 1.5 - abs(4.0 * value - 1.0)))
    return (round(red * 255) << 16) | (round(green * 255) << 8) | round(blue * 255)


def decode_scalar_image(
    message: Image, *, scale: float = 1.0, offset: float = 0.0
) -> list[float]:
    """Decode common one-channel ROS image encodings into floats."""

    encodings = {
        "mono16": ("H", 2),
        "16UC1": ("H", 2),
        "16SC1": ("h", 2),
        "32FC1": ("f", 4),
        "64FC1": ("d", 8),
    }
    if message.encoding not in encodings:
        raise ValueError(f"unsupported scalar image encoding: {message.encoding!r}")
    code, item_size = encodings[message.encoding]
    width = int(message.width)
    height = int(message.height)
    row_step = int(message.step)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if row_step < width * item_size:
        raise ValueError("image row step is shorter than its pixels")
    if len(message.data) < row_step * height:
        raise ValueError("image data buffer is too short")

    byte_order = ">" if message.is_bigendian else "<"
    row_format = f"{byte_order}{width}{code}"
    values: list[float] = []
    for row in range(height):
        decoded = struct.unpack_from(row_format, message.data, row * row_step)
        values.extend(float(value) * scale + offset for value in decoded)
    return values


def decode_scalar_array(
    message: Image, *, scale: float = 1.0, offset: float = 0.0
) -> np.ndarray:
    """Decode a scalar image into a float32 array without per-pixel Python work."""
    encodings = {
        "mono16": ("u2", 2),
        "16UC1": ("u2", 2),
        "16SC1": ("i2", 2),
        "32FC1": ("f4", 4),
        "64FC1": ("f8", 8),
    }
    if message.encoding not in encodings:
        raise ValueError(f"unsupported scalar image encoding: {message.encoding!r}")
    code, item_size = encodings[message.encoding]
    width = int(message.width)
    height = int(message.height)
    row_step = int(message.step)
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    if row_step < width * item_size:
        raise ValueError("image row step is shorter than its pixels")
    if len(message.data) < row_step * height:
        raise ValueError("image data buffer is too short")
    byte_order = ">" if message.is_bigendian else "<"
    view = np.ndarray(
        shape=(height, width),
        dtype=np.dtype(byte_order + code),
        buffer=message.data,
        strides=(row_step, item_size),
    )
    return view.astype(np.float32) * float(scale) + float(offset)


def create_thermal_cloud(
    header: Header,
    points: Iterable[ThermalPoint],
    *,
    color_min_c: float = 10.0,
    color_max_c: float = 60.0,
) -> PointCloud2:
    point_list = list(points)
    data = bytearray(len(point_list) * THERMAL_POINT_STEP)
    for index, point in enumerate(point_list):
        struct.pack_into(
            "<fffffffI",
            data,
            index * THERMAL_POINT_STEP,
            point.x,
            point.y,
            point.z,
            point.temperature_c,
            point.confidence,
            point.pixel_u,
            point.pixel_v,
            temperature_rgb(point.temperature_c, color_min_c, color_max_c),
        )
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(point_list)
    message.fields = list(THERMAL_POINT_FIELDS)
    message.is_bigendian = False
    message.point_step = THERMAL_POINT_STEP
    message.row_step = len(data)
    message.data = bytes(data)
    message.is_dense = False
    return message


def iter_thermal_cloud(message: PointCloud2) -> Iterator[ThermalPoint]:
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "z", "temperature_c", "confidence")
    if any(name not in fields for name in required):
        raise ValueError(
            "thermal cloud requires x, y, z, temperature_c and confidence fields"
        )
    if any(fields[name].datatype != PointField.FLOAT32 for name in required):
        raise ValueError("all thermal cloud fields must use FLOAT32")
    if message.point_step <= 0 or message.row_step < message.width * message.point_step:
        raise ValueError("invalid PointCloud2 layout")
    if len(message.data) < message.row_step * message.height:
        raise ValueError("PointCloud2 data buffer is too short")

    byte_order = ">" if message.is_bigendian else "<"
    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            base = row_offset + column * message.point_step
            values = [
                struct.unpack_from(
                    f"{byte_order}f", message.data, base + fields[name].offset
                )[0]
                for name in required
            ]
            pixel_values = []
            for name in ("pixel_u", "pixel_v"):
                field = fields.get(name)
                pixel_values.append(
                    struct.unpack_from(
                        f"{byte_order}f", message.data, base + field.offset
                    )[0]
                    if field is not None and field.datatype == PointField.FLOAT32
                    else -1.0
                )
            if all(math.isfinite(value) for value in values):
                yield ThermalPoint(*values, *pixel_values)
