"""Safe lifecycle management for one explicit rosbag2 recording session."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
from typing import Callable, Sequence
from uuid import uuid4

from .preflight import PreflightReport


SESSION_NAME = re.compile(r"[^A-Za-z0-9_.-]+")
ALLOWED_STORAGE_IDS = frozenset({"sqlite3", "mcap"})


class SessionError(RuntimeError):
    """A recorder session cannot be started or stopped safely."""


@dataclass(frozen=True)
class SessionPaths:
    root: Path
    session_dir: Path
    bag_dir: Path
    manifest_path: Path
    log_path: Path


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def safe_session_name(value: str) -> str:
    sanitized = SESSION_NAME.sub("-", value.strip()).strip(".-")
    if not sanitized:
        sanitized = "session"
    return sanitized[:80]


def create_session_paths(storage_root: Path, requested_name: str) -> SessionPaths:
    root = storage_root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    identifier = f"{datetime.now().strftime('%Y%m%dT%H%M%S')}-{safe_session_name(requested_name)}-{uuid4().hex[:8]}"
    session_dir = (root / identifier).resolve()
    if root not in session_dir.parents:
        raise SessionError("resolved session path escaped storage root")
    session_dir.mkdir(mode=0o700)
    return SessionPaths(
        root=root,
        session_dir=session_dir,
        bag_dir=session_dir / "bag",
        manifest_path=session_dir / "session.json",
        log_path=session_dir / "rosbag.log",
    )


def _atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _directory_size(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


class BagSession:
    """One bag subprocess, using argument lists instead of a shell command."""

    def __init__(
        self,
        paths: SessionPaths,
        profile_name: str,
        preflight: PreflightReport,
        *,
        storage_id: str = "sqlite3",
        command_runner: Callable[..., subprocess.Popen] = subprocess.Popen,
    ) -> None:
        if storage_id not in ALLOWED_STORAGE_IDS:
            raise SessionError(f"unsupported storage_id: {storage_id}")
        if not preflight.can_start:
            raise SessionError(f"missing required topics: {', '.join(preflight.missing_required)}")
        self.paths = paths
        self.profile_name = profile_name
        self.preflight = preflight
        self.storage_id = storage_id
        self._command_runner = command_runner
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._started_monotonic: float | None = None
        self._started_at: str | None = None
        self._end_reason: str | None = None

    def command(self) -> list[str]:
        topics = list(dict.fromkeys(self.preflight.selected_topics.values()))
        if not topics:
            raise SessionError("recording profile resolved no topics")
        return [
            "ros2",
            "bag",
            "record",
            "--storage",
            self.storage_id,
            "--output",
            str(self.paths.bag_dir),
            *topics,
        ]

    def _manifest(self, status: str) -> dict:
        duration = 0.0 if self._started_monotonic is None else round(time.monotonic() - self._started_monotonic, 3)
        return {
            "schema_version": 1,
            "session_directory": self.paths.session_dir.name,
            "profile": self.profile_name,
            "storage_id": self.storage_id,
            "status": status,
            "started_at": self._started_at,
            "ended_at": utc_now() if status != "recording" else None,
            "duration_seconds": duration,
            "selected_topics": self.preflight.selected_topics,
            "missing_required": list(self.preflight.missing_required),
            "missing_optional": list(self.preflight.missing_optional),
            "free_bytes_at_start": self.preflight.free_bytes,
            "bag_size_bytes": _directory_size(self.paths.bag_dir),
            "end_reason": self._end_reason,
            "command": self.command(),
        }

    def start(self) -> None:
        if self._process is not None:
            raise SessionError("session is already running")
        self._started_at = utc_now()
        self._started_monotonic = time.monotonic()
        _atomic_json(self.paths.manifest_path, self._manifest("recording"))
        log_handle = self.paths.log_path.open("ab")
        try:
            self._process = self._command_runner(
                self.command(),
                stdin=subprocess.DEVNULL,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                shell=False,
                start_new_session=True,
            )
            self._log_handle = log_handle
        except OSError as exc:
            log_handle.close()
            self._end_reason = f"start-failed: {exc}"
            _atomic_json(self.paths.manifest_path, self._manifest("failed"))
            raise SessionError(f"unable to start ros2 bag: {exc}") from exc

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def stop(self, reason: str = "operator-stop", timeout_seconds: float = 8.0) -> dict:
        self._end_reason = reason
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=timeout_seconds)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=timeout_seconds)
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
        status = "limited" if reason.startswith(("max-duration", "max-size")) else "completed"
        manifest = self._manifest(status)
        _atomic_json(self.paths.manifest_path, manifest)
        return manifest

    def update_final_metadata(self, **metadata: object) -> dict:
        """Merge non-sensitive computed results after ``stop`` has completed."""

        if self.is_running():
            raise SessionError("cannot finalize metadata while recording")
        try:
            manifest = json.loads(self.paths.manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise SessionError(f"cannot read session manifest: {exc}") from exc
        manifest.update(metadata)
        _atomic_json(self.paths.manifest_path, manifest)
        return manifest

    def enforce_limits(self, *, max_duration_seconds: float, max_size_bytes: int) -> str | None:
        if not self.is_running():
            return None
        if max_duration_seconds > 0 and self._started_monotonic is not None:
            if time.monotonic() - self._started_monotonic >= max_duration_seconds:
                self.stop("max-duration")
                return "max-duration"
        if max_size_bytes > 0 and _directory_size(self.paths.bag_dir) >= max_size_bytes:
            self.stop("max-size")
            return "max-size"
        return None
