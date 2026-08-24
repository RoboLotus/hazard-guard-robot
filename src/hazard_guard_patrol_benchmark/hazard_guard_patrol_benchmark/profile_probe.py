from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2

from .profile import probe_is_healthy, sample_summary


TOPICS = {
    "scan": (LaserScan, "/scan"),
    "rgb": (Image, "/camera/image_raw"),
    "depth": (Image, "/depth_camera/image_raw"),
    "points": (PointCloud2, "/depth_camera/points"),
    "thermal": (Image, "/thermal_camera/image_raw"),
}
PROCESS_MARKERS = (
    "ign gazebo",
    "nav2_",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "map_server",
    "amcl",
    "parameter_bridge",
    "mission_manager",
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Measure a running Docker simulation profile.")
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--warmup", type=float, default=15.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--render-engine", required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--gpu-enabled", action="store_true")
    return parser.parse_args()


def _process_snapshot() -> tuple[float, float, bool]:
    ticks = 0.0
    rss_pages = 0
    gazebo_alive = False
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            if not any(marker in command for marker in PROCESS_MARKERS):
                continue
            stat = (entry / "stat").read_text(encoding="utf-8")
            # The comm field is wrapped in parentheses and may itself contain
            # spaces (for example ``ign gazebo``), so a naive split shifts all
            # following field indexes.
            fields = stat[stat.rfind(")") + 2 :].split()
            ticks += float(fields[11]) + float(fields[12])
            rss_pages += int(fields[21])
            gazebo_alive = gazebo_alive or "ign gazebo" in command
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            continue
    return ticks, rss_pages * os.sysconf("SC_PAGE_SIZE") / 1024**2, gazebo_alive


def _gpu_sample() -> tuple[float | None, float | None]:
    try:
        completed = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            check=True,
            text=True,
            timeout=2.0,
        )
        first = completed.stdout.strip().splitlines()[0]
        utilization, memory = (float(value.strip()) for value in first.split(",")[:2])
        return utilization, memory
    except (OSError, subprocess.SubprocessError, IndexError, ValueError):
        return None, None


class ProfileProbe(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_docker_profile_probe")
        self.counts = {name: 0 for name in TOPICS}
        self.sim_time: float | None = None
        self.create_subscription(
            Clock, "/clock", self._clock, qos_profile_sensor_data
        )
        self._subscriptions = [
            self.create_subscription(
                message_type,
                topic,
                lambda message, key=name: self._count(key, message),
                qos_profile_sensor_data,
            )
            for name, (message_type, topic) in TOPICS.items()
        ]

    def _clock(self, message: Clock) -> None:
        self.sim_time = float(message.clock.sec) + float(message.clock.nanosec) / 1e9

    def _count(self, name: str, message: Any) -> None:
        self.counts[name] += 1
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is not None:
            self.sim_time = float(stamp.sec) + float(stamp.nanosec) / 1e9

    def reset_counts(self) -> None:
        for name in self.counts:
            self.counts[name] = 0


def _spin_for(node: Node, duration: float) -> None:
    deadline = time.monotonic() + max(0.0, duration)
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_once(node, timeout_sec=min(0.2, remaining))


def main() -> None:
    args = _arguments()
    rclpy.init()
    node = ProfileProbe()
    try:
        _spin_for(node, args.warmup)
        node.reset_counts()
        sim_start = node.sim_time
        wall_start = time.monotonic()
        previous_wall = wall_start
        previous_ticks, _, _ = _process_snapshot()
        cpu_samples: list[float] = []
        memory_samples: list[float] = []
        gpu_samples: list[float] = []
        gpu_memory_samples: list[float] = []
        gazebo_alive = False
        clock_ticks = float(os.sysconf("SC_CLK_TCK"))
        deadline = wall_start + max(1.0, args.duration)
        while time.monotonic() < deadline:
            _spin_for(node, min(1.0, deadline - time.monotonic()))
            now = time.monotonic()
            ticks, rss_mib, current_alive = _process_snapshot()
            elapsed = max(now - previous_wall, 1e-6)
            cpu_samples.append(max(0.0, (ticks - previous_ticks) / clock_ticks / elapsed * 100.0))
            memory_samples.append(rss_mib)
            gazebo_alive = gazebo_alive or current_alive
            gpu, gpu_memory = _gpu_sample()
            if gpu is not None:
                gpu_samples.append(gpu)
            if gpu_memory is not None:
                gpu_memory_samples.append(gpu_memory)
            previous_wall = now
            previous_ticks = ticks

        wall_seconds = time.monotonic() - wall_start
        sim_end = node.sim_time
        sim_seconds = max(0.0, (sim_end or 0.0) - (sim_start or 0.0))
        document: dict[str, Any] = {
            "schema_version": 1,
            "profile_id": args.profile_id,
            "render_engine": args.render_engine,
            "gui": args.gui,
            "gpu_enabled": args.gpu_enabled,
            "wall_seconds": round(wall_seconds, 4),
            "sim_seconds": round(sim_seconds, 4),
            "real_time_factor": round(sim_seconds / wall_seconds, 4),
            "gazebo_alive": gazebo_alive,
            "topic_counts": node.counts,
            "topic_rates_wall_hz": {
                name: round(count / wall_seconds, 4) for name, count in node.counts.items()
            },
            "topic_rates_sim_hz": {
                name: round(count / sim_seconds, 4) if sim_seconds else 0.0
                for name, count in node.counts.items()
            },
            "process_cpu_percent": sample_summary(cpu_samples),
            "process_rss_mib": sample_summary(memory_samples),
            "gpu_utilization_percent": sample_summary(gpu_samples),
            "gpu_memory_mib": sample_summary(gpu_memory_samples),
        }
        document["healthy"] = probe_is_healthy(document)
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(output)
        print(output)
        print(json.dumps(document, ensure_ascii=False))
        if not document["healthy"]:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
