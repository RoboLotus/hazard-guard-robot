from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_launch_exposes_environment_and_world_arguments() -> None:
    text = (PACKAGE_ROOT / "launch/patrol_benchmark.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'DeclareLaunchArgument("world_id"' in text
    assert 'DeclareLaunchArgument("simulation_env_path"' in text
    assert 'DeclareLaunchArgument("storage_path"' in text


def test_default_config_is_simulation_only() -> None:
    text = (PACKAGE_ROOT / "config/benchmark_defaults.yaml").read_text(
        encoding="utf-8"
    )
    assert "use_sim_time: true" in text
    assert "ground_truth_topic: /odom" in text
