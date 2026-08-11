from types import SimpleNamespace

import pytest

from hazard_guard_mission_manager.schedule import (
    PatrolSchedule,
    REPEAT_COUNT,
    REPEAT_FOREVER,
    REPEAT_ONCE,
    REPEAT_UNTIL_TIME,
)


def request(**overrides):
    values = {
        "repeat_mode": REPEAT_ONCE,
        "repeat_count": 1,
        "repeat_interval_sec": 0.0,
        "start_at_unix_ms": 0,
        "end_at_unix_ms": 0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_once_runs_exactly_one_cycle():
    schedule = PatrolSchedule.from_request(request())

    assert schedule.total_cycles == 1
    assert schedule.should_continue(0) is True
    assert schedule.should_continue(1) is False


def test_count_uses_requested_total_cycles():
    schedule = PatrolSchedule.from_request(
        request(repeat_mode=REPEAT_COUNT, repeat_count=4)
    )

    assert schedule.total_cycles == 4
    assert schedule.should_continue(3) is True
    assert schedule.should_continue(4) is False


def test_forever_has_no_total_cycle_limit():
    schedule = PatrolSchedule.from_request(
        request(repeat_mode=REPEAT_FOREVER, repeat_count=0)
    )

    assert schedule.total_cycles == 0
    assert schedule.should_continue(10_000) is True


def test_wall_clock_deadline_is_inclusive():
    schedule = PatrolSchedule.from_request(
        request(
            repeat_mode=REPEAT_UNTIL_TIME,
            start_at_unix_ms=1_000,
            end_at_unix_ms=2_000,
        )
    )

    assert schedule.deadline_reached(1_999) is False
    assert schedule.deadline_reached(2_000) is True


def test_end_time_must_follow_start_time():
    with pytest.raises(ValueError, match="종료 시각"):
        PatrolSchedule.from_request(
            request(
                repeat_mode=REPEAT_UNTIL_TIME,
                start_at_unix_ms=2_000,
                end_at_unix_ms=1_000,
            )
        )
