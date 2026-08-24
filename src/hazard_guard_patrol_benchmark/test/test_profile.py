from hazard_guard_patrol_benchmark.profile import (
    aggregate_profiles,
    percentile,
    probe_health_failures,
    probe_is_healthy,
)


def _probe(profile_id: str, *, gui: bool, rtf: float, healthy: bool = True) -> dict:
    rates = {
        "scan": 9.8,
        "rgb": 9.7,
        "depth": 9.7,
        "points": 9.6,
        "thermal": 8.4,
    }
    if not healthy:
        rates["depth"] = 0.0
    return {
        "profile_id": profile_id,
        "gui": gui,
        "gpu_enabled": "gpu" in profile_id,
        "render_engine": "ogre" if "gpu" in profile_id else "ogre2",
        "gazebo_alive": True,
        "sim_seconds": 20.0,
        "real_time_factor": rtf,
        "topic_rates_sim_hz": rates,
        "process_cpu_percent": {"median": 250.0},
        "process_rss_mib": {"median": 1200.0},
        "gpu_utilization_percent": {"median": 30.0},
    }


def test_nearest_rank_percentile() -> None:
    assert percentile([1, 2, 3, 4, 5], 0.95) == 5.0


def test_probe_requires_every_sensor_and_live_gazebo() -> None:
    assert probe_is_healthy(_probe("cpu", gui=False, rtf=0.5))
    assert not probe_is_healthy(_probe("cpu", gui=False, rtf=0.5, healthy=False))


def test_health_failure_explains_the_rejected_stream() -> None:
    failures = probe_health_failures(
        _probe("gpu-gui", gui=True, rtf=0.25, healthy=False)
    )
    assert failures == ["depth_rate_below_5hz:0.0000"]


def test_selects_fastest_fully_healthy_profile_per_gui_mode() -> None:
    summary = aggregate_profiles(
        [
            _probe("cpu-headless", gui=False, rtf=0.45),
            _probe("gpu-headless", gui=False, rtf=0.62),
            _probe("cpu-gui", gui=True, rtf=0.20),
            _probe("gpu-gui", gui=True, rtf=0.35),
        ]
    )
    assert summary["recommended"] == {
        "automated": "gpu-headless",
        "visual": "gpu-gui",
    }


def test_rejected_profile_keeps_measurements_for_comparison() -> None:
    rejected = _probe("gpu-gui", gui=True, rtf=0.25, healthy=False)
    summary = aggregate_profiles([rejected])
    profile = summary["profiles"]["gpu-gui"]

    assert profile["all_healthy"] is False
    assert profile["real_time_factor"]["median"] == 0.25
    assert profile["failed_runs"][0]["failures"] == [
        "depth_rate_below_5hz:0.0000"
    ]
