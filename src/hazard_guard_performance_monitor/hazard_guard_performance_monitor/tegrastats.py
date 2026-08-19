from __future__ import annotations

import re
import shutil
import subprocess
import threading
from typing import Any


GPU_PATTERN = re.compile(r"GR3D_FREQ\s+(\d+)%")
RAM_PATTERN = re.compile(r"RAM\s+(\d+)/(\d+)MB")
TEMP_PATTERN = re.compile(r"([A-Za-z0-9_]+)@([0-9.]+)C")
POWER_PATTERN = re.compile(r"([A-Za-z0-9_]+)\s+(\d+)mW(?:/(\d+)mW)?")


def parse_tegrastats_line(line: str) -> dict[str, Any]:
    result: dict[str, Any] = {}
    gpu = GPU_PATTERN.search(line)
    if gpu:
        result["gpu_percent"] = float(gpu.group(1))
    ram = RAM_PATTERN.search(line)
    if ram:
        used, total = int(ram.group(1)), int(ram.group(2))
        result["tegrastats_ram_used_mb"] = float(used)
        result["tegrastats_ram_percent"] = (
            round(100.0 * used / total, 3) if total else 0.0
        )
    temperatures = {
        name.lower(): float(value)
        for name, value in TEMP_PATTERN.findall(line)
    }
    if temperatures:
        result["temperatures_c"] = temperatures
    power = {
        name.lower(): float(current)
        for name, current, _average in POWER_PATTERN.findall(line)
    }
    if power:
        result["power_mw"] = power
    return result


class TegrastatsReader:
    """Keep the latest tegrastats sample without blocking a ROS callback."""

    def __init__(self, interval_ms: int = 1000) -> None:
        self._interval_ms = max(100, int(interval_ms))
        self._process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None

    @property
    def available(self) -> bool:
        return shutil.which("tegrastats") is not None

    def start(self) -> bool:
        if not self.available or self._process is not None:
            return False
        self._process = subprocess.Popen(
            ["tegrastats", "--interval", str(self._interval_ms)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def _read_loop(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            parsed = parse_tegrastats_line(line)
            if parsed:
                with self._lock:
                    self._latest = parsed

    def snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def stop(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)
