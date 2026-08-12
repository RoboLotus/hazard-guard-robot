#!/usr/bin/env python3
"""Bound the live 3D map workload without affecting SLAM Toolbox or Nav2.

The node sits only in the visualization cloud path.  It caps points and input
rate before the RTAB-Map cloud assembler, and rate-limits the cumulative map
sent to downstream consumers.  Under sustained Jetson load it degrades or
pauses this path; RGB-D odometry, the RTAB-Map database, and 2D navigation keep
running independently.
"""

from __future__ import annotations

from collections import deque
import glob
import json
import os
from pathlib import Path
import shutil
import time
from typing import Deque, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


MODE_RANK = {"normal": 0, "high_load": 1, "critical": 2}


class LoadPolicy:
    """Moving-average load policy with delayed, hysteretic recovery."""

    def __init__(self, window_samples: int, recovery_seconds: float):
        self.samples: Deque[Dict[str, float]] = deque(
            maxlen=max(3, window_samples)
        )
        self.recovery_seconds = max(1.0, recovery_seconds)
        self.mode = "normal"
        self._recovery_candidate_since: Optional[float] = None

    @staticmethod
    def _at_or_above(
        values: Dict[str, float], limits: Dict[str, float]
    ) -> bool:
        return any(
            values.get(name, 0.0) >= limit
            for name, limit in limits.items()
        )

    @staticmethod
    def _below(values: Dict[str, float], limits: Dict[str, float]) -> bool:
        return all(
            values.get(name, 0.0) < limit
            for name, limit in limits.items()
        )

    def add(
        self, sample: Dict[str, float], now: float
    ) -> Tuple[str, Dict[str, float]]:
        self.samples.append(sample)
        averaged = {
            name: sum(item.get(name, 0.0) for item in self.samples)
            / len(self.samples)
            for name in sample
        }
        if len(self.samples) < 3:
            return self.mode, averaged

        critical_limits = {
            "cpu": 94.0,
            "gpu": 98.0,
            "memory": 94.0,
            "temperature": 87.0,
            "disk": 95.0,
        }
        high_limits = {
            "cpu": 82.0,
            "gpu": 90.0,
            "memory": 88.0,
            "temperature": 80.0,
            "disk": 85.0,
        }
        desired = "normal"
        if self._at_or_above(averaged, critical_limits):
            desired = "critical"
        elif self._at_or_above(averaged, high_limits):
            desired = "high_load"

        if MODE_RANK[desired] > MODE_RANK[self.mode]:
            self.mode = desired
            self._recovery_candidate_since = None
            return self.mode, averaged

        if MODE_RANK[desired] == MODE_RANK[self.mode]:
            self._recovery_candidate_since = None
            return self.mode, averaged

        exit_limits = (
            {
                "cpu": 85.0,
                "gpu": 92.0,
                "memory": 90.0,
                "temperature": 82.0,
                "disk": 92.0,
            }
            if self.mode == "critical"
            else {
                "cpu": 72.0,
                "gpu": 75.0,
                "memory": 82.0,
                "temperature": 75.0,
                "disk": 80.0,
            }
        )
        if not self._below(averaged, exit_limits):
            self._recovery_candidate_since = None
            return self.mode, averaged

        if self._recovery_candidate_since is None:
            self._recovery_candidate_since = now
        elif now - self._recovery_candidate_since >= self.recovery_seconds:
            self.mode = desired
            self._recovery_candidate_since = None
        return self.mode, averaged


class SystemLoadReader:
    def __init__(self, storage_path: str):
        self.storage_path = Path(storage_path).expanduser()
        self._previous_cpu: Optional[Tuple[int, int]] = None
        gpu_candidates = [
            "/sys/devices/platform/bus@0/17000000.gpu/load",
            "/sys/devices/gpu.0/load",
            *glob.glob("/sys/class/devfreq/*/load"),
        ]
        self._gpu_load_paths = [
            Path(path) for path in gpu_candidates if Path(path).exists()
        ]

    @staticmethod
    def _read_number(path: Path) -> Optional[float]:
        try:
            return float(path.read_text(encoding="ascii").strip())
        except (OSError, TypeError, ValueError):
            return None

    def _cpu_percent(self) -> float:
        try:
            fields = (
                Path("/proc/stat")
                .read_text(encoding="ascii")
                .splitlines()[0]
                .split()
            )
            values = [int(value) for value in fields[1:]]
        except (OSError, ValueError, IndexError):
            return 0.0
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        current = (idle, total)
        if self._previous_cpu is None:
            self._previous_cpu = current
            return 0.0
        idle_delta = idle - self._previous_cpu[0]
        total_delta = total - self._previous_cpu[1]
        self._previous_cpu = current
        if total_delta <= 0:
            return 0.0
        return 100.0 * (1.0 - idle_delta / total_delta)

    @staticmethod
    def _memory_percent() -> float:
        values: Dict[str, int] = {}
        try:
            lines = (
                Path("/proc/meminfo")
                .read_text(encoding="ascii")
                .splitlines()
            )
            for line in lines:
                name, value = line.split(":", 1)
                values[name] = int(value.strip().split()[0])
        except (OSError, ValueError, IndexError):
            return 0.0
        total = values.get("MemTotal", 0)
        available = values.get("MemAvailable", 0)
        if total <= 0:
            return 0.0
        return 100.0 * (total - available) / total

    def _gpu_percent(self) -> float:
        loads = []
        for path in self._gpu_load_paths:
            value = self._read_number(path)
            if value is not None:
                loads.append(value / 10.0 if value > 100.0 else value)
        return max(loads, default=0.0)

    def _temperature_celsius(self) -> float:
        temperatures = []
        for name in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
            value = self._read_number(Path(name))
            if value is None:
                continue
            value = value / 1000.0 if value > 1000.0 else value
            if 0.0 < value < 150.0:
                temperatures.append(value)
        return max(temperatures, default=0.0)

    def _disk_percent(self) -> float:
        try:
            usage = shutil.disk_usage(self.storage_path)
        except OSError:
            return 0.0
        if usage.total <= 0:
            return 0.0
        return 100.0 * usage.used / usage.total

    def read(self) -> Dict[str, float]:
        return {
            "cpu": self._cpu_percent(),
            "gpu": self._gpu_percent(),
            "memory": self._memory_percent(),
            "temperature": self._temperature_celsius(),
            "disk": self._disk_percent(),
        }


def evenly_sample_cloud(
    message: PointCloud2, target_points: int
) -> PointCloud2:
    """Evenly sample point indexes while preserving PointCloud2 fields."""
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    if width < 0 or height < 0:
        raise ValueError("PointCloud2 dimensions must not be negative")
    if point_step <= 0:
        raise ValueError("PointCloud2 point_step must be positive")

    minimum_row_step = width * point_step
    if row_step < minimum_row_step:
        raise ValueError(
            "PointCloud2 row_step is smaller than width * point_step"
        )

    required_bytes = row_step * height
    if len(message.data) < required_bytes:
        raise ValueError(
            "PointCloud2 data buffer is shorter than row_step * height"
        )

    total_points = width * height
    target_points = max(1, int(target_points))
    if total_points <= target_points:
        return message

    source = memoryview(message.data)
    output_data = bytearray(target_points * point_step)
    for output_index in range(target_points):
        source_index = output_index * total_points // target_points
        row, column = divmod(source_index, width)
        source_offset = row * row_step + column * point_step
        destination_offset = output_index * point_step
        output_data[
            destination_offset:destination_offset + point_step
        ] = source[source_offset:source_offset + point_step]

    result = PointCloud2()
    result.header = message.header
    result.height = 1
    result.width = target_points
    result.fields = message.fields
    result.is_bigendian = message.is_bigendian
    result.point_step = point_step
    result.row_step = target_points * point_step
    result.data = bytes(output_data)
    result.is_dense = message.is_dense
    return result


class AdaptiveCloudGuard(Node):
    def __init__(self):
        super().__init__("hazard_guard_adaptive_cloud_guard")
        self.declare_parameter("normal_points", 3000)
        self.declare_parameter("high_load_points", 1500)
        self.declare_parameter("normal_input_hz", 8.0)
        self.declare_parameter("high_load_input_hz", 4.0)
        self.declare_parameter("normal_surface_hz", 1.0)
        self.declare_parameter("high_load_surface_hz", 0.5)
        self.declare_parameter("load_window_seconds", 8)
        self.declare_parameter("recovery_seconds", 15.0)
        self.declare_parameter("storage_path", os.getcwd())

        self._profiles = {
            "normal": {
                "points": int(self.get_parameter("normal_points").value),
                "input_hz": float(self.get_parameter("normal_input_hz").value),
                "surface_hz": float(
                    self.get_parameter("normal_surface_hz").value
                ),
            },
            "high_load": {
                "points": int(
                    self.get_parameter("high_load_points").value
                ),
                "input_hz": float(
                    self.get_parameter("high_load_input_hz").value
                ),
                "surface_hz": float(
                    self.get_parameter("high_load_surface_hz").value
                ),
            },
            "critical": {"points": 0, "input_hz": 0.0, "surface_hz": 0.2},
        }
        window = int(self.get_parameter("load_window_seconds").value)
        recovery = float(self.get_parameter("recovery_seconds").value)
        storage_path = str(self.get_parameter("storage_path").value)
        self._policy = LoadPolicy(window, recovery)
        self._load_reader = SystemLoadReader(storage_path)
        self._averaged_load: Dict[str, float] = {}

        self._last_input_publish = 0.0
        self._last_surface_publish = 0.0
        self._latest_surface: Optional[PointCloud2] = None
        self._surface_dirty = False
        self._input_points = 0
        self._output_points = 0
        self._surface_points = 0
        self._rate_drops = 0
        self._critical_drops = 0
        self._malformed_drops = 0

        self._frame_publisher = self.create_publisher(
            PointCloud2, "output", qos_profile_sensor_data
        )
        self._surface_publisher = self.create_publisher(
            PointCloud2, "surface_output", qos_profile_sensor_data
        )
        self._surface_compat_publisher = self.create_publisher(
            PointCloud2, "surface_compat_output", qos_profile_sensor_data
        )
        self._status_publisher = self.create_publisher(String, "status", 1)
        self.create_subscription(
            PointCloud2, "input", self._on_frame, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "surface_input",
            self._on_surface,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._monitor_load)
        self.create_timer(0.1, self._publish_surface_if_due)
        self.create_timer(5.0, self._publish_status)
        normal = self._profiles["normal"]
        high = self._profiles["high_load"]
        self.get_logger().info(
            "Adaptive 3D cloud guard started: "
            f"normal={normal['points']}@{normal['input_hz']:.1f}Hz, "
            f"high_load={high['points']}@{high['input_hz']:.1f}Hz, "
            "critical pauses map integration"
        )

    @property
    def _profile(self) -> Dict[str, float]:
        return self._profiles[self._policy.mode]

    def _on_frame(self, message: PointCloud2) -> None:
        self._input_points = int(message.width) * int(message.height)
        if self._policy.mode == "critical":
            self._critical_drops += 1
            return
        now = time.monotonic()
        input_hz = self._profile["input_hz"]
        if (
            input_hz > 0.0
            and now - self._last_input_publish < 1.0 / input_hz
        ):
            self._rate_drops += 1
            return
        try:
            target_points = int(self._profile["points"])
            filtered = evenly_sample_cloud(message, target_points)
        except (ValueError, TypeError) as error:
            self._malformed_drops += 1
            self.get_logger().error(f"Malformed PointCloud2 dropped: {error}")
            return
        self._last_input_publish = now
        self._output_points = int(filtered.width) * int(filtered.height)
        self._frame_publisher.publish(filtered)

    def _on_surface(self, message: PointCloud2) -> None:
        self._latest_surface = message
        self._surface_dirty = True
        self._surface_points = int(message.width) * int(message.height)

    def _publish_surface_if_due(self) -> None:
        if not self._surface_dirty or self._latest_surface is None:
            return
        now = time.monotonic()
        surface_hz = self._profile["surface_hz"]
        if (
            surface_hz <= 0.0
            or now - self._last_surface_publish < 1.0 / surface_hz
        ):
            return
        self._surface_publisher.publish(self._latest_surface)
        self._surface_compat_publisher.publish(self._latest_surface)
        self._latest_surface = None
        self._surface_dirty = False
        self._last_surface_publish = now

    def _monitor_load(self) -> None:
        previous_mode = self._policy.mode
        mode, self._averaged_load = self._policy.add(
            self._load_reader.read(), time.monotonic()
        )
        if mode != previous_mode:
            values = ", ".join(
                f"{name}={value:.1f}"
                for name, value in self._averaged_load.items()
            )
            self.get_logger().warn(
                "3D cloud quality mode changed: "
                f"{previous_mode} -> {mode} ({values})"
            )
            self._publish_status()

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "mode": self._policy.mode,
                "profile": self._profile,
                "load": {
                    name: round(value, 1)
                    for name, value in self._averaged_load.items()
                },
                "input_points": self._input_points,
                "output_points": self._output_points,
                "surface_points": self._surface_points,
                "rate_drops": self._rate_drops,
                "critical_drops": self._critical_drops,
                "malformed_drops": self._malformed_drops,
            },
            separators=(",", ":"),
        )
        self._status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = AdaptiveCloudGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
