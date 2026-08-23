"""Read-only recording preflight checks.

The recorder resolves aliases against the current ROS graph before creating a
bag.  This avoids recording a topic name that belongs to a simulator while the
physical driver is publishing on a different name.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
from typing import Iterable

from .profiles import RecordingProfile


class PreflightError(RuntimeError):
    """A requested session is not safe or complete enough to start."""


@dataclass(frozen=True)
class PreflightReport:
    profile_name: str
    selected_topics: dict[str, str]
    missing_required: tuple[str, ...]
    missing_optional: tuple[str, ...]
    free_bytes: int

    @property
    def can_start(self) -> bool:
        return not self.missing_required


def resolve_topics(profile: RecordingProfile, available_topics: Iterable[str]) -> tuple[dict[str, str], tuple[str, ...], tuple[str, ...]]:
    """Select the first visible alias for every configured topic."""

    available = set(available_topics)
    selected: dict[str, str] = {}
    missing_required: list[str] = []
    missing_optional: list[str] = []
    for spec in profile.topics:
        topic = next((candidate for candidate in spec.candidates if candidate in available), None)
        if topic is not None:
            selected[spec.identifier] = topic
        elif spec.required:
            missing_required.append(spec.identifier)
        else:
            missing_optional.append(spec.identifier)
    return selected, tuple(missing_required), tuple(missing_optional)


def run_preflight(
    profile: RecordingProfile,
    available_topics: Iterable[str],
    storage_root: Path,
    *,
    minimum_free_bytes: int,
    allow_experimental: bool = False,
) -> PreflightReport:
    """Validate topic availability, experimental status, and free disk space."""

    if profile.experimental and not allow_experimental:
        raise PreflightError(
            f"{profile.name} is experimental; set allow_experimental:=true after confirming physical topics"
        )
    storage_root = storage_root.expanduser()
    storage_root.mkdir(parents=True, exist_ok=True)
    free_bytes = shutil.disk_usage(storage_root).free
    if minimum_free_bytes < 0:
        raise PreflightError("minimum_free_bytes must not be negative")
    if free_bytes < minimum_free_bytes:
        raise PreflightError(
            f"insufficient storage: {free_bytes} bytes free, {minimum_free_bytes} bytes required"
        )
    selected, missing_required, missing_optional = resolve_topics(profile, available_topics)
    return PreflightReport(
        profile_name=profile.name,
        selected_topics=selected,
        missing_required=missing_required,
        missing_optional=missing_optional,
        free_bytes=free_bytes,
    )
