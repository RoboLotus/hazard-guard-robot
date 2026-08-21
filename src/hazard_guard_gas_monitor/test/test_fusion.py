from hazard_guard_gas_monitor.fusion import fuse_risk, thermal_identity


def test_reads_real_and_simulated_thermal_identity() -> None:
    assert thermal_identity(
        "thermal_trend:bunker_waste_pile:watch:hotspot", 52.0
    ) == ("bunker_waste_pile", "watch", "hotspot")
    assert thermal_identity("gazebo:bunker_waste_pile", 71.3) == (
        "bunker_waste_pile",
        "warning",
        "simulated_temperature_screening",
    )
    assert thermal_identity("gazebo:bunker_waste_pile", 52.0) == (
        "bunker_waste_pile",
        "watch",
        "simulated_temperature_screening",
    )


def test_single_weak_signal_is_watch() -> None:
    decision = fuse_risk(
        voc_abnormal=True,
        co_warning=False,
        co_critical=False,
        localized_voc=False,
        thermal_status="normal",
        same_zone=True,
    )
    assert decision.level == "watch"


def test_two_weak_signals_in_same_zone_become_warning() -> None:
    decision = fuse_risk(
        voc_abnormal=True,
        co_warning=False,
        co_critical=False,
        localized_voc=False,
        thermal_status="watch",
        same_zone=True,
    )
    assert decision.level == "warning"


def test_different_zone_thermal_does_not_confirm_gas() -> None:
    decision = fuse_risk(
        voc_abnormal=True,
        co_warning=False,
        co_critical=False,
        localized_voc=False,
        thermal_status="critical",
        same_zone=False,
    )
    assert decision.level == "watch"


def test_three_independent_signals_are_critical() -> None:
    decision = fuse_risk(
        voc_abnormal=True,
        co_warning=True,
        co_critical=False,
        localized_voc=True,
        thermal_status="watch",
        same_zone=True,
    )
    assert decision.level == "critical"


def test_any_absolute_critical_is_critical() -> None:
    decision = fuse_risk(
        voc_abnormal=False,
        co_warning=True,
        co_critical=True,
        localized_voc=False,
        thermal_status="normal",
        same_zone=False,
    )
    assert decision.level == "critical"

    thermal = fuse_risk(
        voc_abnormal=False,
        co_warning=False,
        co_critical=False,
        localized_voc=False,
        thermal_status="critical",
        same_zone=True,
    )
    assert thermal.level == "critical"
