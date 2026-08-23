from pathlib import Path


ROOT = Path(__file__).parents[2]
SIMULATION = ROOT / "hazard_guard_simulation" / "launch"


def test_all_patrol_launches_start_the_performance_monitor():
    for filename in (
        "navigation.launch.py",
        "localization.launch.py",
        "physical_patrol.launch.py",
    ):
        source = (SIMULATION / filename).read_text(encoding="utf-8")
        assert '"use_performance_monitor"' in source
        assert 'package="hazard_guard_performance_monitor"' in source
        assert 'executable="performance_monitor"' in source
        assert '"performance_storage_path"' in source
