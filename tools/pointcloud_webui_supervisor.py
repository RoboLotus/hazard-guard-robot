#!/usr/bin/env python3
"""Run the WebUI backend and summarize load while 3D mapping is active."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import os
import signal
import subprocess
import time
from typing import Any

from hazard_guard_performance_monitor.procfs import (
    cpu_percentages,
    read_cpu_ticks,
    read_memory,
)
from hazard_guard_performance_monitor.statistics import summarize_values
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


STATUS_TOPIC = "/hazard_guard/rtabmap/cloud_guard/status"


def format_duration(seconds: float) -> str:
    total = max(0, round(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def process_group_exists(group_id: int) -> bool:
    try:
        os.killpg(group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def managed_ros_groups(parent_pid: int) -> set[int]:
    """Return validated ROS launch groups directly owned by the backend."""
    children_path = Path(f"/proc/{parent_pid}/task/{parent_pid}/children")
    try:
        child_ids = children_path.read_text(encoding="ascii").split()
    except OSError:
        return set()
    groups: set[int] = set()
    for child_id in child_ids:
        if not child_id.isdigit():
            continue
        try:
            command = (
                Path(f"/proc/{child_id}/cmdline")
                .read_bytes()
                .replace(b"\0", b" ")
                .decode("utf-8", errors="replace")
            )
            child_pid = int(child_id)
            group_id = os.getpgid(child_pid)
        except (OSError, ProcessLookupError, ValueError):
            continue
        if (
            "ros2 launch hazard_guard_simulation" in command
            and group_id == child_pid
        ):
            groups.add(group_id)
    return groups


def stop_process_group(group_id: int) -> None:
    for stop_signal, timeout in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 3.0),
        (signal.SIGKILL, 2.0),
    ):
        if not process_group_exists(group_id):
            return
        try:
            os.killpg(group_id, stop_signal)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not process_group_exists(group_id):
                return
            time.sleep(0.1)


def stop_backend(process: subprocess.Popen[Any]) -> bool:
    """Stop Uvicorn, escalating like a user's second Ctrl+C if required."""
    if process.poll() is not None:
        return False
    ros_groups = managed_ros_groups(process.pid)
    forced = False
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        forced = True
        print(
            "\n백엔드 종료가 지연되어 두 번째 Ctrl+C로 강제 종료합니다.",
            flush=True,
        )
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            print(
                "백엔드가 응답하지 않아 Uvicorn 프로세스를 종료합니다.",
                flush=True,
            )
            process.kill()
            process.wait(timeout=5)

    # Normally FastAPI's lifespan shuts down the WebUI-managed ROS group.
    # Clean only the exact, validated group captured before forced shutdown if
    # Uvicorn could not finish that lifecycle work.
    for group_id in ros_groups:
        if process_group_exists(group_id):
            forced = True
            stop_process_group(group_id)
    return forced


class GuardMonitor(Node):
    def __init__(self) -> None:
        super().__init__("hazard_guard_pointcloud_webui_summary")
        self.latest: dict[str, Any] | None = None
        self.received_at: float | None = None
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.latest = payload
            self.received_at = time.monotonic()

    def mapping_active(self, now: float) -> bool:
        return (
            self.latest is not None
            and self.received_at is not None
            and now - self.received_at <= 6.0
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend-python", required=True)
    parser.add_argument("--backend-dir", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--points", required=True, type=int)
    parser.add_argument("--input-hz", required=True, type=float)
    parser.add_argument("--surface-hz", required=True, type=float)
    parser.add_argument("--voxel-size", required=True, type=float)
    parser.add_argument("--decimation", required=True, type=int)
    return parser.parse_args()


def print_summary(
    args: argparse.Namespace,
    cpu_samples: list[float],
    ram_samples: list[float],
    modes: Counter[str],
    output_points: list[int],
    surface_points: list[int],
    active_started_at: float | None,
    active_finished_at: float | None,
) -> None:
    width = 58
    print("\n" + "=" * width)
    print("  HazardGuard 3D PointCloud 부하 테스트 결과")
    print("=" * width)
    print(f"  프로필              : {args.profile}")
    print(f"  포인트/프레임 설정  : {args.points:,}")
    print(f"  입력 주기 설정      : {args.input_hz:g} Hz")
    print(f"  지도 전송 주기      : {args.surface_hz:g} Hz")
    print(f"  Voxel 크기          : {args.voxel_size:g} m")
    print(f"  Decimation          : {args.decimation}")
    if not cpu_samples:
        print("-" * width)
        print("  측정 결과 없음: 3D mapping 상태 토픽을 수신하지 못했습니다.")
        print("  WebUI에서 '2D + RGB-D 3D -> 새 맵 생성'을 실행했는지 확인하세요.")
        print("=" * width)
        return

    active_duration = 0.0
    if active_started_at is not None and active_finished_at is not None:
        active_duration = active_finished_at - active_started_at
    observed_summary = summarize_values(output_points)
    observed_points = round(float(observed_summary["mean"] or 0.0))
    peak_surface = max(surface_points, default=0)
    mode_text = ", ".join(
        f"{name} {count}초" for name, count in sorted(modes.items())
    )
    print("-" * width)
    print(f"  측정 시간           : {format_duration(active_duration)}")
    print(f"  유효 샘플           : {len(cpu_samples):,}개")
    cpu_summary = summarize_values(cpu_samples)
    ram_summary = summarize_values(ram_samples)
    print(
        "  CPU                 : "
        f"평균 {cpu_summary['mean']:.1f}% / "
        f"중앙값 {cpu_summary['median']:.1f}% / "
        f"P95 {cpu_summary['p95']:.1f}% / 최대 {cpu_summary['max']:.1f}%"
    )
    print(
        "  RAM                 : "
        f"평균 {ram_summary['mean']:.1f}% / "
        f"중앙값 {ram_summary['median']:.1f}% / "
        f"P95 {ram_summary['p95']:.1f}% / 최대 {ram_summary['max']:.1f}%"
    )
    print(f"  실제 출력 포인트    : 평균 {observed_points:,} points/frame")
    print(f"  최대 누적 포인트    : {peak_surface:,}")
    print(f"  Guard 모드          : {mode_text or '확인 불가'}")
    print("=" * width)


def main() -> int:
    args = parse_args()
    command = [
        args.backend_python,
        "-m",
        "uvicorn",
        "app.main:app",
        "--host",
        args.host,
        "--port",
        str(args.port),
    ]

    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    rclpy.init()
    # rclpy installs its own SIGINT handler during init. Replace it after
    # initialization so Ctrl+C can first stop Uvicorn cleanly, then print the
    # accumulated benchmark summary.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    monitor = GuardMonitor()
    process = subprocess.Popen(
        command,
        cwd=args.backend_dir,
        env=os.environ.copy(),
        start_new_session=True,
    )
    previous_cpu = read_cpu_ticks()
    next_sample = time.monotonic() + 1.0
    cpu_samples: list[float] = []
    ram_samples: list[float] = []
    output_points: list[int] = []
    surface_points: list[int] = []
    modes: Counter[str] = Counter()
    active_started_at: float | None = None
    active_finished_at: float | None = None

    forced_shutdown = False
    try:
        while process.poll() is None and not stop_requested:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            now = time.monotonic()
            if now < next_sample:
                continue
            current_cpu = read_cpu_ticks()
            current_cpu_percent = cpu_percentages(
                previous_cpu,
                current_cpu,
            ).get("cpu", 0.0)
            previous_cpu = current_cpu
            if monitor.mapping_active(now):
                status = monitor.latest or {}
                if active_started_at is None:
                    active_started_at = now
                    print(
                        "\n3D mapping 감지: CPU/RAM 평균 측정을 시작합니다.",
                        flush=True,
                    )
                active_finished_at = now
                cpu_samples.append(current_cpu_percent)
                ram_samples.append(read_memory()["used_percent"])
                modes[str(status.get("mode") or "unknown")] += 1
                try:
                    output_points.append(int(status.get("output_points") or 0))
                    surface_points.append(
                        int(status.get("surface_points") or 0)
                    )
                except (TypeError, ValueError):
                    pass
            while next_sample <= now:
                next_sample += 1.0
    finally:
        forced_shutdown = stop_backend(process)
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print_summary(
        args,
        cpu_samples,
        ram_samples,
        modes,
        output_points,
        surface_points,
        active_started_at,
        active_finished_at,
    )
    if forced_shutdown:
        print("  참고: 종료 지연으로 백엔드/ROS 정리 신호를 단계적으로 적용했습니다.")
    return process.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
