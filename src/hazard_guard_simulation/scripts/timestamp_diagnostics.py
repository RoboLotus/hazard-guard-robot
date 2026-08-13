#!/usr/bin/env python3
"""Measure physical sensor timestamp skew and TF availability for 3D SLAM."""

from __future__ import annotations

import csv
import io
import json
import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


SENSOR_QOS = QoSProfile(
    # Diagnostics only need the newest sample. A deep queue would retain large
    # RGB-D/cloud messages and compete with RTAB-Map on Jetson memory.
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return float(ordered[index])


@dataclass
class TopicStats:
    window_size: int
    received: int = 0
    last_stamp_ns: int | None = None
    ages_sec: deque[float] = field(init=False)
    latest_odom_offsets_sec: deque[float] = field(init=False)
    tf_results: deque[bool] = field(init=False)

    def __post_init__(self) -> None:
        self.ages_sec = deque(maxlen=self.window_size)
        self.latest_odom_offsets_sec = deque(maxlen=self.window_size)
        self.tf_results = deque(maxlen=self.window_size)

    def add(
        self,
        *,
        stamp_ns: int,
        now_ns: int,
        odom_stamp_ns: int | None,
        has_tf: bool,
    ) -> None:
        self.received += 1
        self.last_stamp_ns = stamp_ns
        self.ages_sec.append((now_ns - stamp_ns) / 1e9)
        self.tf_results.append(has_tf)
        if odom_stamp_ns is not None:
            self.latest_odom_offsets_sec.append(
                (stamp_ns - odom_stamp_ns) / 1e9
            )

    def summary(self) -> dict[str, Any]:
        ages = list(self.ages_sec)
        offsets = list(self.latest_odom_offsets_sec)
        tf_results = list(self.tf_results)
        return {
            "received": self.received,
            "window_samples": len(ages),
            "last_stamp_ns": self.last_stamp_ns,
            "age_sec": _statistics(ages),
            # This is a reception-order diagnostic, not time-interpolated
            # odometry. It compares against the most recently received odom.
            "offset_to_latest_odom_sec": _statistics(offsets),
            "tf_available_ratio": (
                sum(tf_results) / len(tf_results) if tf_results else None
            ),
        }


def _statistics(values: list[float]) -> dict[str, float | None]:
    return {
        "mean": sum(values) / len(values) if values else None,
        "max_abs": max((abs(value) for value in values), default=None),
        "p95_abs": percentile([abs(value) for value in values], 0.95),
        "latest": values[-1] if values else None,
    }


def render_csv(payload: dict[str, Any]) -> str:
    output = io.StringIO()
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(
        [
            "topic",
            "received",
            "window_samples",
            "age_mean_sec",
            "age_p95_abs_sec",
            "latest_odom_offset_mean_sec",
            "latest_odom_offset_p95_abs_sec",
            "tf_available_ratio",
        ]
    )
    for topic, values in payload["topics"].items():
        writer.writerow(
            [
                topic,
                values["received"],
                values["window_samples"],
                values["age_sec"]["mean"],
                values["age_sec"]["p95_abs"],
                values["offset_to_latest_odom_sec"]["mean"],
                values["offset_to_latest_odom_sec"]["p95_abs"],
                values["tf_available_ratio"],
            ]
        )
    return output.getvalue()


def render_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# HazardGuard 3D SLAM timestamp diagnostics",
        "",
        f"Generated (Unix ns): `{payload['generated_at_ns']}`",
        "",
        "| Topic | Samples | Mean age (s) | P95 | Odom offset (s) | TF |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for topic, values in payload["topics"].items():
        age = values["age_sec"]
        offset = values["offset_to_latest_odom_sec"]
        lines.append(
            "| {topic} | {samples} | {age} | {p95} | {offset} | {tf} |".format(
                topic=topic,
                samples=values["window_samples"],
                age=_format_number(age["mean"]),
                p95=_format_number(age["p95_abs"]),
                offset=_format_number(offset["mean"]),
                tf=_format_number(values["tf_available_ratio"]),
            )
        )
    lines.extend(
        [
            "",
            "> A negative age means the sensor timestamp is ahead of ROS "
            "time. Choose an offset only after measuring a stable skew on "
            "the robot.",
            "",
        ]
    )
    return "\n".join(lines)


def _format_number(value: float | None) -> str:
    return "-" if value is None else f"{value:.4f}"


class TimestampDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_timestamp_diagnostics")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("window_size", 300)
        self.declare_parameter("publish_interval_sec", 2.0)
        self.declare_parameter("report_interval_sec", 30.0)
        self.declare_parameter("report_dir", "")
        self._target_frame = str(self.get_parameter("target_frame").value)
        window_size = max(10, int(self.get_parameter("window_size").value))
        topic_names = ("rgb", "depth", "camera_info", "odom", "scan", "cloud")
        self._stats = {
            name: TopicStats(window_size) for name in topic_names
        }
        self._latest_odom_stamp_ns: int | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        self._publisher = self.create_publisher(
            String,
            "/hazard_guard/rtabmap/sync_status",
            STATUS_QOS,
        )
        self._subscriptions = [
            self.create_subscription(
                Image, "rgb", self._callback("rgb"), SENSOR_QOS
            ),
            self.create_subscription(
                Image, "depth", self._callback("depth"), SENSOR_QOS
            ),
            self.create_subscription(
                CameraInfo,
                "camera_info",
                self._callback("camera_info"),
                SENSOR_QOS,
            ),
            self.create_subscription(
                Odometry, "odom", self._callback("odom"), SENSOR_QOS
            ),
            self.create_subscription(
                LaserScan, "scan", self._callback("scan"), SENSOR_QOS
            ),
            self.create_subscription(
                PointCloud2, "cloud", self._callback("cloud"), SENSOR_QOS
            ),
        ]
        publish_interval = max(
            0.2,
            float(self.get_parameter("publish_interval_sec").value),
        )
        self.create_timer(publish_interval, self._publish)
        self._report_dir = str(self.get_parameter("report_dir").value).strip()
        report_interval = float(
            self.get_parameter("report_interval_sec").value
        )
        if self._report_dir and report_interval > 0.0:
            self.create_timer(max(1.0, report_interval), self._write_reports)

    def _callback(self, name: str):
        def callback(message: Any) -> None:
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            now_ns = self.get_clock().now().nanoseconds
            if name == "odom":
                self._latest_odom_stamp_ns = stamp_ns
            frame_id = str(message.header.frame_id)
            has_tf = self._has_transform(frame_id, stamp_ns)
            self._stats[name].add(
                stamp_ns=stamp_ns,
                now_ns=now_ns,
                odom_stamp_ns=self._latest_odom_stamp_ns,
                has_tf=has_tf,
            )

        return callback

    def _has_transform(self, frame_id: str, stamp_ns: int) -> bool:
        if not frame_id:
            return False
        if frame_id == self._target_frame:
            return True
        return self._tf_buffer.can_transform(
            self._target_frame,
            frame_id,
            Time(nanoseconds=stamp_ns),
            # Diagnostics run in the same executor as this TF listener. Never
            # wait here: blocking would prevent queued TF from being handled.
            timeout=Duration(),
        )

    def _payload(self) -> dict[str, Any]:
        return {
            "generated_at_ns": self.get_clock().now().nanoseconds,
            "target_frame": self._target_frame,
            "topics": {
                name: stats.summary() for name, stats in self._stats.items()
            },
        }

    def _publish(self) -> None:
        message = String()
        message.data = json.dumps(
            self._payload(),
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self._publisher.publish(message)

    def _write_reports(self) -> None:
        try:
            report_dir = Path(self._report_dir).expanduser()
            report_dir.mkdir(parents=True, exist_ok=True)
            payload = self._payload()
            files = {
                "timestamp_diagnostics.json": json.dumps(
                    payload,
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n",
                "timestamp_diagnostics.csv": render_csv(payload),
                "timestamp_diagnostics.md": render_markdown(payload),
            }
            for filename, content in files.items():
                temporary = report_dir / f".{filename}.tmp"
                temporary.write_text(content, encoding="utf-8")
                temporary.replace(report_dir / filename)
        except Exception as exc:
            self.get_logger().error(f"Failed to write timestamp report: {exc}")


def main() -> None:
    rclpy.init()
    node = TimestampDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
