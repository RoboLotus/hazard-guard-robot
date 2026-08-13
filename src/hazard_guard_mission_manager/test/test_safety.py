from hazard_guard_mission_manager.safety import SafetyPauseLatch


def test_disabled_latch_never_pauses_existing_patrols() -> None:
    latch = SafetyPauseLatch(enabled=False)

    assert latch.update(SafetyPauseLatch.STOP, "person close") is False
    assert latch.is_paused() is False


def test_stop_and_sensor_fault_pause_when_enabled() -> None:
    latch = SafetyPauseLatch(enabled=True)

    assert latch.update(SafetyPauseLatch.STOP, "person close") is True
    assert latch.is_paused() is True
    assert latch.update(SafetyPauseLatch.SENSOR_FAULT, "camera stale") is False
    assert latch.snapshot() == (SafetyPauseLatch.SENSOR_FAULT, "camera stale")


def test_clear_releases_pause_once() -> None:
    latch = SafetyPauseLatch(enabled=True)
    latch.update(SafetyPauseLatch.STOP)

    assert latch.update(0, "clear hold complete") is True
    assert latch.is_paused() is False
    assert latch.update(0, "still clear") is False
