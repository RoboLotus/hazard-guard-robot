from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
from typing import Any


ENVIRONMENT_VARIABLE = "HAZARD_GUARD_SIMULATION_ENV"


@dataclass(frozen=True)
class WorldAssets:
    repository_root: Path
    world_id: str
    world_name: str
    world_path: Path
    map_path: Path
    heat_sources_path: Path | None
    patrol_script_path: Path | None
    spawn: dict[str, float]
    catalog_entry: dict[str, Any]


def _git_commit(repository: Path) -> str | None:
    try:
        return subprocess.run(
            [
                "git",
                "-c",
                f"safe.directory={repository}",
                "-C",
                str(repository),
                "rev-parse",
                "HEAD",
            ],
            capture_output=True,
            check=True,
            text=True,
            timeout=2.0,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def repository_commit(repository: Path) -> str | None:
    return _git_commit(repository)


def resolve_environment_root(configured: str = "") -> Path:
    candidates: list[Path] = []
    if configured.strip():
        candidates.append(Path(configured).expanduser())
    environment = os.getenv(ENVIRONMENT_VARIABLE, "").strip()
    if environment:
        candidates.append(Path(environment).expanduser())
    workspace = os.getenv("HAZARD_GUARD_WORKSPACE", "").strip()
    if workspace:
        workspace_path = Path(workspace).expanduser()
        candidates.extend(
            [
                workspace_path.parent / "Simulation_env",
                workspace_path / "Simulation_env",
            ]
        )
    current = Path.cwd()
    candidates.extend(
        [
            current / "Simulation_env",
            current.parent / "Simulation_env",
            current.parent.parent / "Simulation_env",
        ]
    )

    visited: set[Path] = set()
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved in visited:
            continue
        visited.add(resolved)
        if (resolved / "sims" / "catalog.json").is_file():
            return resolved
    searched = ", ".join(str(path) for path in visited)
    raise FileNotFoundError(
        "Simulation_env 저장소를 찾을 수 없습니다. "
        f"{ENVIRONMENT_VARIABLE} 또는 simulation_env_path를 설정하세요. "
        f"검색 경로: {searched}"
    )


def _asset_path(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} 경로가 catalog.json에 없습니다")
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{label} 경로가 Simulation_env 밖을 가리킵니다") from exc
    if not path.is_file():
        raise FileNotFoundError(f"{label} 파일을 찾을 수 없습니다: {path}")
    return path


def load_world_assets(root: Path, world_id: str) -> WorldAssets:
    catalog_path = root / "sims" / "catalog.json"
    document = json.loads(catalog_path.read_text(encoding="utf-8"))
    worlds = document.get("worlds")
    if not isinstance(worlds, dict) or world_id not in worlds:
        available = ", ".join(sorted(worlds or {}))
        raise KeyError(f"알 수 없는 world_id '{world_id}' (사용 가능: {available})")
    entry = worlds[world_id]
    if not isinstance(entry, dict):
        raise ValueError(f"world '{world_id}' 항목이 객체가 아닙니다")

    heat_sources_path = None
    if entry.get("heat_sources"):
        heat_sources_path = _asset_path(
            root, entry["heat_sources"], "heat_sources"
        )
    map_path = _asset_path(root, entry.get("map"), "map")
    patrol_candidate = (
        root / "gazebo" / "config" / "patrol" / f"run_patrol_{world_id}.sh"
    ).resolve()
    patrol_script_path = patrol_candidate if patrol_candidate.is_file() else None
    spawn_raw = entry.get("spawn") or {}
    spawn = {
        key: float(spawn_raw.get(key, 0.0))
        for key in ("x", "y", "z", "yaw")
    }
    return WorldAssets(
        repository_root=root,
        world_id=world_id,
        world_name=str(entry.get("world_name") or world_id),
        world_path=_asset_path(root, entry.get("world"), "world"),
        map_path=map_path,
        heat_sources_path=heat_sources_path,
        patrol_script_path=patrol_script_path,
        spawn=spawn,
        catalog_entry=entry,
    )


def load_heat_source_ids(path: Path | None) -> set[str]:
    if path is None:
        return set()
    document = json.loads(path.read_text(encoding="utf-8"))
    sources = document.get("sources")
    if not isinstance(sources, list):
        raise ValueError(f"heat source profile에 sources 배열이 없습니다: {path}")
    return {
        str(item["detection_id"])
        for item in sources
        if isinstance(item, dict) and item.get("detection_id")
    }
