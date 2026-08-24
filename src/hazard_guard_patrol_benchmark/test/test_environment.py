import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from hazard_guard_patrol_benchmark.environment import (
    load_heat_source_ids,
    load_world_assets,
    repository_commit,
    resolve_environment_root,
)


def _repository(root: Path) -> None:
    (root / "sims").mkdir()
    (root / "gazebo/worlds").mkdir(parents=True)
    (root / "gazebo/maps").mkdir(parents=True)
    (root / "gazebo/config/heat_sources").mkdir(parents=True)
    (root / "gazebo/config/patrol").mkdir(parents=True)
    (root / "gazebo/worlds/test.sdf").write_text("<sdf/>", encoding="utf-8")
    (root / "gazebo/maps/test.yaml").write_text("image: test.pgm\n", encoding="utf-8")
    heat = root / "gazebo/config/heat_sources/test.json"
    heat.write_text(
        json.dumps({"sources": [{"detection_id": "hot-01"}]}),
        encoding="utf-8",
    )
    (root / "gazebo/config/patrol/run_patrol_test.sh").write_text(
        "#!/bin/sh\n", encoding="utf-8"
    )
    (root / "sims/catalog.json").write_text(
        json.dumps(
            {
                "worlds": {
                    "test": {
                        "world": "gazebo/worlds/test.sdf",
                        "world_name": "test_world",
                        "map": "gazebo/maps/test.yaml",
                        "heat_sources": "gazebo/config/heat_sources/test.json",
                        "spawn": {"x": 1, "y": 2, "z": 0.05, "yaw": 0.5},
                    }
                }
            }
        ),
        encoding="utf-8",
    )


def test_resolves_configured_repository_and_world_assets(tmp_path: Path) -> None:
    _repository(tmp_path)
    root = resolve_environment_root(str(tmp_path))
    assets = load_world_assets(root, "test")
    assert assets.world_name == "test_world"
    assert assets.spawn["y"] == 2.0
    assert assets.patrol_script_path is not None
    assert load_heat_source_ids(assets.heat_sources_path) == {"hot-01"}


def test_rejects_asset_path_outside_repository(tmp_path: Path) -> None:
    _repository(tmp_path)
    catalog = tmp_path / "sims/catalog.json"
    document = json.loads(catalog.read_text(encoding="utf-8"))
    document["worlds"]["test"]["world"] = "../../outside.sdf"
    catalog.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="밖을 가리킵니다"):
        load_world_assets(tmp_path, "test")


def test_repository_commit_allows_docker_bind_mount_ownership(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    captured: list[str] = []

    def fake_run(command: list[str], **_: object) -> SimpleNamespace:
        captured.extend(command)
        return SimpleNamespace(stdout="abc123\n")

    monkeypatch.setattr(
        "hazard_guard_patrol_benchmark.environment.subprocess.run",
        fake_run,
    )

    assert repository_commit(tmp_path) == "abc123"
    assert f"safe.directory={tmp_path}" in captured
