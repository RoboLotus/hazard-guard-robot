from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import socket
import subprocess
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .procfs import (
    cpu_percentages,
    process_usage,
    read_cpu_ticks,
    read_memory,
    read_processes,
)
from .report import PerformanceSession, default_storage_root
from .tegrastats import TegrastatsReader


COLLECTING_STATES = {
    "preparing",
    "running",
    "executing",
    "aligning",
    "dwelling",
    "safety_paused",
}
TERMINAL_STATES = {"completed", "failed", "canceled"}


def _git_commit(workspace: str) -> str | None:
    if not workspace:
        return None
    try:
        return subprocess.run(
            ["git", "-C", workspace, "rev-parse", "HEAD"],
            capture_output=True,
            check=True,
            text=True,
            timeout=2.0,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


class PerformanceMonitor(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_performance_monitor")
        self.declare_parameter("sample_interval_sec", 1.0)
        self.declare_parameter("storage_path", "")
        interval = max(
            0.25,
            float(self.get_parameter("sample_interval_sec").value),
        )
        configured_path = str(self.get_parameter("storage_path").value).strip()
        self._storage_root = (
            Path(configured_path).expanduser().resolve()
            if configured_path
            else default_storage_root()
        )
        self._storage_root.mkdir(parents=True, exist_ok=True)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            "/hazard_guard/mission/status",
            self._on_mission,
            qos,
        )
        self._mission: dict[str, Any] = {}
        self._session: PerformanceSession | None = None
        self._previous_cpu = read_cpu_ticks()
        self._previous_processes = read_processes()
        self._previous_sample_at = time.monotonic()
        self._active_elapsed_sec = 0.0
        self._tegrastats = TegrastatsReader(round(interval * 1000))
        self._tegrastats.start()
        self.create_timer(interval, self._sample)
        self.get_logger().info(
            f"Performance reports: {self._storage_root}"
        )

    def _platform_metadata(self) -> dict[str, Any]:
        return {
            "hostname": socket.gethostname(),
            "architecture": platform.machine(),
            "platform": platform.platform(),
            "cpu_count": os.cpu_count(),
            "tegrastats": self._tegrastats.available,
            "git_commit": _git_commit(os.getenv("HAZARD_GUARD_WORKSPACE", "")),
        }

    def _on_mission(self, message: String) -> None:
        try:
            mission = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(mission, dict):
            return
        status = str(mission.get("status") or "idle")
        mission_id = str(mission.get("mission_id") or "")
        current_id = str(self._mission.get("mission_id") or "")
        if (
            self._session is not None
            and mission_id
            and current_id
            and mission_id != current_id
        ):
            self._session.finalize("interrupted")
            self._session = None
        self._mission = mission
        if (
            self._session is None
            and status in COLLECTING_STATES
            and mission_id
        ):
            self._session = PerformanceSession(
                self._storage_root,
                mission,
                self._platform_metadata(),
            )
            self._active_elapsed_sec = 0.0
            self.get_logger().info(
                f"Performance collection started: {mission_id}"
            )
        if self._session is not None and status in TERMINAL_STATES:
            summary = self._session.finalize(status)
            self.get_logger().info(
                f"Performance report completed: {summary['id']}"
            )
            self._session = None

    def _sample(self) -> None:
        now = time.monotonic()
        current_cpu = read_cpu_ticks()
        current_processes = read_processes()
        elapsed = max(0.001, now - self._previous_sample_at)
        cpu = cpu_percentages(self._previous_cpu, current_cpu)
        processes = process_usage(
            self._previous_processes,
            current_processes,
            elapsed_sec=elapsed,
        )
        self._previous_cpu = current_cpu
        self._previous_processes = current_processes
        self._previous_sample_at = now
        if self._session is None:
            return
        phase = str(self._mission.get("status") or "unknown")
        if phase not in COLLECTING_STATES:
            return
        self._active_elapsed_sec += elapsed
        memory = read_memory()
        sample = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "elapsed_sec": round(self._active_elapsed_sec, 3),
            "phase": phase,
            "current_waypoint": self._mission.get("current_index"),
            "current_cycle": self._mission.get("current_cycle"),
            "cpu": {
                "total_percent": cpu.get("cpu"),
                "cores": {
                    name: value for name, value in cpu.items() if name != "cpu"
                },
            },
            "memory": {
                "used_percent": memory["used_percent"],
                "used_mb": round(memory["used_bytes"] / 1024 / 1024, 3),
                "available_mb": round(
                    memory["available_bytes"] / 1024 / 1024, 3
                ),
                "swap_used_mb": round(
                    memory["swap_used_bytes"] / 1024 / 1024, 3
                ),
            },
            "processes": processes,
            "jetson": self._tegrastats.snapshot() or {},
        }
        self._session.append(sample)

    def destroy_node(self) -> bool:
        if self._session is not None:
            self._session.finalize("interrupted")
            self._session = None
        self._tegrastats.stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PerformanceMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
