"""Validated, explicit rosbag topic contracts.

Profiles deliberately list topics instead of using ``ros2 bag record --all``.
That keeps YOLO/debug traffic and unknown high-bandwidth topics out of field bags.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re
from typing import Any, Mapping


PROFILE_NAME = re.compile(r"^[a-z][a-z0-9-]{1,63}$")
TOPIC_NAME = re.compile(r"^/[A-Za-z0-9_/]+$")


@dataclass(frozen=True)
class TopicSpec:
    identifier: str
    candidates: tuple[str, ...]
    required: bool


@dataclass(frozen=True)
class RecordingProfile:
    name: str
    description: str
    experimental: bool
    topics: tuple[TopicSpec, ...]


class ProfileError(ValueError):
    """A profile document cannot be safely used for a recording command."""


def _topic_spec(raw: Mapping[str, Any]) -> TopicSpec:
    identifier = str(raw.get("id", "")).strip()
    candidates = raw.get("candidates")
    if not identifier or not isinstance(candidates, list) or not candidates:
        raise ProfileError("topic requires a non-empty id and candidates")
    values = tuple(str(item).strip() for item in candidates)
    if any(not TOPIC_NAME.fullmatch(value) for value in values):
        raise ProfileError(f"invalid ROS topic candidate for {identifier!r}")
    return TopicSpec(
        identifier=identifier,
        candidates=values,
        required=bool(raw.get("required", False)),
    )


def load_profile_document(path: Path) -> dict[str, dict[str, Any]]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProfileError(f"could not load profile document: {exc}") from exc
    profiles = document.get("profiles") if isinstance(document, dict) else None
    if not isinstance(profiles, dict):
        raise ProfileError("profile document requires a profiles object")
    return {str(name): value for name, value in profiles.items() if isinstance(value, dict)}


def resolve_profile(name: str, definitions: Mapping[str, Mapping[str, Any]]) -> RecordingProfile:
    """Resolve inheritance while preserving topic order and rejecting cycles."""

    if not PROFILE_NAME.fullmatch(name):
        raise ProfileError("invalid profile name")
    chain: list[str] = []

    def visit(profile_name: str) -> list[TopicSpec]:
        if profile_name in chain:
            raise ProfileError("profile inheritance cycle")
        raw = definitions.get(profile_name)
        if raw is None:
            raise ProfileError(f"unknown profile {profile_name!r}")
        chain.append(profile_name)
        inherited: list[TopicSpec] = []
        parent = raw.get("extends")
        if parent:
            inherited = visit(str(parent))
        own_raw = raw.get("topics", [])
        if not isinstance(own_raw, list):
            raise ProfileError(f"topics for {profile_name!r} must be a list")
        if any(not isinstance(item, dict) for item in own_raw):
            raise ProfileError(f"topics for {profile_name!r} must contain objects")
        combined = [*inherited, *(_topic_spec(item) for item in own_raw)]
        chain.pop()
        return combined

    raw = definitions.get(name)
    if raw is None:
        raise ProfileError(f"unknown profile {name!r}")
    topics_by_id: dict[str, TopicSpec] = {}
    for topic in visit(name):
        topics_by_id[topic.identifier] = topic
    return RecordingProfile(
        name=name,
        description=str(raw.get("description", "")).strip(),
        experimental=bool(raw.get("experimental", False)),
        topics=tuple(topics_by_id.values()),
    )
