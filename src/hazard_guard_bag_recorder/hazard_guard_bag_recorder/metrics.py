"""Small in-process measurements saved beside a rosbag session."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class DriveMetrics:
    samples: int = 0
    distance_m: float = 0.0
    max_speed_mps: float = 0.0
    total_speed_mps: float = 0.0
    stopped_seconds: float = 0.0
    latest_mission_status: str | None = None
    _last_position: tuple[float, float] | None = None
    _last_timestamp: float | None = None

    def observe_odometry(self, x: float, y: float, linear_x: float, linear_y: float, timestamp: float) -> None:
        speed = math.hypot(linear_x, linear_y)
        if self._last_position is not None:
            self.distance_m += math.dist(self._last_position, (x, y))
        if self._last_timestamp is not None:
            elapsed = max(0.0, timestamp - self._last_timestamp)
            if speed <= 0.02:
                self.stopped_seconds += elapsed
        self._last_position = (x, y)
        self._last_timestamp = timestamp
        self.samples += 1
        self.total_speed_mps += speed
        self.max_speed_mps = max(self.max_speed_mps, speed)

    def as_dict(self) -> dict:
        return {
            "odometry_samples": self.samples,
            "distance_m": round(self.distance_m, 3),
            "average_speed_mps": round(self.total_speed_mps / self.samples, 3) if self.samples else 0.0,
            "max_speed_mps": round(self.max_speed_mps, 3),
            "stopped_seconds": round(self.stopped_seconds, 3),
            "latest_mission_status": self.latest_mission_status,
        }
