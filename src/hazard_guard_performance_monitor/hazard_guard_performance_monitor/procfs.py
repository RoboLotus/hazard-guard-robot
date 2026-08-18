from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class CpuTicks:
    idle: int
    total: int


@dataclass(frozen=True)
class ProcessTicks:
    pid: int
    start_ticks: int
    cpu_ticks: int
    rss_bytes: int
    threads: int
    read_bytes: int
    write_bytes: int
    command: str
    label: str


DEFAULT_PROCESS_PATTERNS = {
    "nav2_controller": ("controller_server",),
    "nav2_planner": ("planner_server",),
    "nav2_behavior": ("behavior_server",),
    "nav2_navigator": ("bt_navigator",),
    "nav2_waypoint": ("waypoint_follower",),
    "nav2_velocity": ("velocity_smoother",),
    "mission_manager": ("hazard_guard_mission_manager",),
    "slam_toolbox": ("slam_toolbox",),
    "rtabmap": ("rtabmap", "map_assembler"),
    "yolo": ("hazard_guard_person_detection", "ultralytics"),
    "safety": ("hazard_guard_safety", "cmd_vel_safety_gate"),
    "fastapi": ("uvicorn", "app.main:app"),
    "gazebo": ("ign gazebo", "ignition-gazebo"),
}


def _cpu_ticks(fields: list[str]) -> CpuTicks:
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return CpuTicks(idle=idle, total=sum(values))


def read_cpu_ticks(proc_root: Path = Path("/proc")) -> dict[str, CpuTicks]:
    result: dict[str, CpuTicks] = {}
    for line in (proc_root / "stat").read_text(encoding="ascii").splitlines():
        fields = line.split()
        name = fields[0]
        if name == "cpu" or (name.startswith("cpu") and name[3:].isdigit()):
            result[name] = _cpu_ticks(fields)
    return result


def cpu_percentages(
    previous: dict[str, CpuTicks],
    current: dict[str, CpuTicks],
) -> dict[str, float]:
    percentages: dict[str, float] = {}
    for name, now in current.items():
        before = previous.get(name)
        if before is None:
            continue
        total_delta = now.total - before.total
        idle_delta = now.idle - before.idle
        if total_delta <= 0:
            continue
        percentages[name] = round(
            100.0 * (1.0 - idle_delta / total_delta),
            3,
        )
    return percentages


def read_memory(proc_root: Path = Path("/proc")) -> dict[str, float]:
    values: dict[str, int] = {}
    for line in (
        (proc_root / "meminfo").read_text(encoding="ascii").splitlines()
    ):
        name, value = line.split(":", 1)
        values[name] = int(value.strip().split()[0]) * 1024
    total = values.get("MemTotal", 0)
    available = values.get("MemAvailable", 0)
    used = max(0, total - available)
    swap_total = values.get("SwapTotal", 0)
    swap_free = values.get("SwapFree", 0)
    return {
        "total_bytes": float(total),
        "available_bytes": float(available),
        "used_bytes": float(used),
        "used_percent": round(100.0 * used / total, 3) if total else 0.0,
        "swap_used_bytes": float(max(0, swap_total - swap_free)),
    }


def process_label(
    command: str,
    patterns: dict[str, Iterable[str]] = DEFAULT_PROCESS_PATTERNS,
) -> str | None:
    lowered = command.lower()
    for label, candidates in patterns.items():
        if any(candidate.lower() in lowered for candidate in candidates):
            return label
    return None


def _read_io(path: Path) -> tuple[int, int]:
    values: dict[str, int] = {}
    try:
        for line in path.read_text(encoding="ascii").splitlines():
            key, value = line.split(":", 1)
            values[key] = int(value.strip())
    except (OSError, ValueError):
        pass
    return values.get("read_bytes", 0), values.get("write_bytes", 0)


def read_processes(
    proc_root: Path = Path("/proc"),
    patterns: dict[str, Iterable[str]] = DEFAULT_PROCESS_PATTERNS,
) -> dict[tuple[int, int], ProcessTicks]:
    page_size = os.sysconf("SC_PAGE_SIZE")
    result: dict[tuple[int, int], ProcessTicks] = {}
    for directory in proc_root.iterdir():
        if not directory.name.isdigit():
            continue
        try:
            command = (
                (directory / "cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode("utf-8", errors="replace")
                .strip()
            )
            label = process_label(command, patterns)
            if not label:
                continue
            stat = (directory / "stat").read_text(encoding="ascii")
            closing = stat.rfind(")")
            fields = stat[closing + 2:].split()
            pid = int(directory.name)
            cpu_ticks = int(fields[11]) + int(fields[12])
            threads = int(fields[17])
            start_ticks = int(fields[19])
            rss_bytes = max(0, int(fields[21])) * page_size
            read_bytes, write_bytes = _read_io(directory / "io")
        except (OSError, IndexError, ValueError):
            continue
        key = (pid, start_ticks)
        result[key] = ProcessTicks(
            pid=pid,
            start_ticks=start_ticks,
            cpu_ticks=cpu_ticks,
            rss_bytes=rss_bytes,
            threads=threads,
            read_bytes=read_bytes,
            write_bytes=write_bytes,
            command=command,
            label=label,
        )
    return result


def process_usage(
    previous: dict[tuple[int, int], ProcessTicks],
    current: dict[tuple[int, int], ProcessTicks],
    *,
    elapsed_sec: float,
    clock_ticks_per_sec: int | None = None,
) -> list[dict[str, float | int | str]]:
    if elapsed_sec <= 0:
        return []
    ticks_per_sec = clock_ticks_per_sec or os.sysconf("SC_CLK_TCK")
    usage: list[dict[str, float | int | str]] = []
    for key, now in current.items():
        before = previous.get(key)
        if before is None:
            continue
        cpu_delta = max(0, now.cpu_ticks - before.cpu_ticks)
        usage.append(
            {
                "pid": now.pid,
                "label": now.label,
                "cpu_core_percent": round(
                    100.0 * cpu_delta / ticks_per_sec / elapsed_sec,
                    3,
                ),
                "rss_mb": round(now.rss_bytes / 1024 / 1024, 3),
                "threads": now.threads,
                "read_bytes_delta": max(0, now.read_bytes - before.read_bytes),
                "write_bytes_delta": max(
                    0,
                    now.write_bytes - before.write_bytes,
                ),
            }
        )
    return usage
