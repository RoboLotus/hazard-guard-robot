import math

import pytest

from hazard_guard_safety_supervisor.policy import (
    PersonObservation,
    PersonSafetyPolicy,
    PolicyConfig,
    SafetyState,
)


def observation(distance_m: float, *, confidence: float = 0.9) -> PersonObservation:
    return PersonObservation(
        confidence=confidence,
        distance_m=distance_m,
        distance_valid=True,
    )


def evaluate(
    policy: PersonSafetyPolicy,
    now_sec: float,
    observations=(),
    *,
    last_update_sec=None,
    autonomous=True,
    sensor_healthy=True,
    sensor_reason="",
):
    if last_update_sec is None:
        last_update_sec = now_sec
    return policy.evaluate(
        now_sec=now_sec,
        observations=observations,
        last_detector_update_sec=last_update_sec,
        autonomous=autonomous,
        sensor_healthy=sensor_healthy,
        sensor_reason=sensor_reason,
    )


def test_distance_zones_escalate_immediately() -> None:
    policy = PersonSafetyPolicy(PolicyConfig())

    assert evaluate(policy, 0.0, [observation(2.2)]).state == SafetyState.CAUTION
    assert evaluate(policy, 0.1, [observation(1.4)]).state == SafetyState.SLOW
    assert evaluate(policy, 0.2, [observation(0.5)]).state == SafetyState.STOP


def test_invalid_distance_is_never_clear() -> None:
    policy = PersonSafetyPolicy(PolicyConfig())
    unknown = PersonObservation(confidence=0.9, distance_valid=False)

    result = evaluate(policy, 1.0, [unknown])

    assert result.state == SafetyState.SLOW
    assert result.person_count == 1
    assert not result.distance_valid
    assert math.isnan(result.nearest_distance_m)


def test_invalid_distance_does_not_release_stop() -> None:
    policy = PersonSafetyPolicy(PolicyConfig())
    assert evaluate(policy, 0.0, [observation(0.5)]).state == SafetyState.STOP

    unknown = PersonObservation(confidence=0.9, distance_valid=False)
    assert evaluate(policy, 0.1, [unknown]).state == SafetyState.STOP


def test_close_valid_person_dominates_another_invalid_distance() -> None:
    policy = PersonSafetyPolicy(PolicyConfig())
    unknown = PersonObservation(confidence=0.9, distance_valid=False)

    result = evaluate(policy, 0.0, [unknown, observation(0.5)])

    assert result.state == SafetyState.STOP
    assert result.nearest_distance_m == pytest.approx(0.5)


def test_stale_detector_is_fault_only_while_autonomous() -> None:
    autonomous = PersonSafetyPolicy(PolicyConfig(detection_timeout_sec=0.5))
    manual = PersonSafetyPolicy(PolicyConfig(detection_timeout_sec=0.5))

    assert evaluate(
        autonomous, 1.0, last_update_sec=0.0, autonomous=True
    ).state == SafetyState.SENSOR_FAULT
    assert evaluate(
        manual, 1.0, last_update_sec=0.0, autonomous=False
    ).state == SafetyState.CLEAR


def test_unhealthy_rgb_depth_is_fault_while_autonomous() -> None:
    policy = PersonSafetyPolicy(PolicyConfig())

    result = evaluate(
        policy,
        1.0,
        sensor_healthy=False,
        sensor_reason="depth registration is not verified",
    )

    assert result.state == SafetyState.SENSOR_FAULT
    assert result.reason == "depth registration is not verified"


def test_clear_requires_continuous_hold_after_stop() -> None:
    policy = PersonSafetyPolicy(PolicyConfig(clear_hold_sec=2.0))
    assert evaluate(policy, 0.0, [observation(0.5)]).state == SafetyState.STOP

    assert evaluate(policy, 1.0).state == SafetyState.STOP
    assert evaluate(policy, 2.9).state == SafetyState.STOP
    assert evaluate(policy, 3.0).state == SafetyState.CLEAR


def test_new_hazard_resets_clear_hold_timer() -> None:
    policy = PersonSafetyPolicy(PolicyConfig(clear_hold_sec=2.0))
    assert evaluate(policy, 0.0, [observation(0.5)]).state == SafetyState.STOP
    assert evaluate(policy, 1.0).state == SafetyState.STOP
    assert evaluate(policy, 2.0, [observation(1.4)]).state == SafetyState.SLOW
    assert evaluate(policy, 2.1).state == SafetyState.SLOW
    assert evaluate(policy, 4.0).state == SafetyState.SLOW
    assert evaluate(policy, 4.1).state == SafetyState.CLEAR


def test_far_person_also_requires_clear_hold_after_fault() -> None:
    policy = PersonSafetyPolicy(
        PolicyConfig(detection_timeout_sec=0.5, clear_hold_sec=1.0)
    )
    assert evaluate(
        policy, 1.0, last_update_sec=0.0, autonomous=True
    ).state == SafetyState.SENSOR_FAULT

    assert evaluate(policy, 1.1, [observation(4.0)]).state == SafetyState.SENSOR_FAULT
    assert evaluate(policy, 2.0, [observation(4.0)]).state == SafetyState.SENSOR_FAULT
    assert evaluate(policy, 2.1, [observation(4.0)]).state == SafetyState.CLEAR


def test_hysteresis_prevents_boundary_chatter() -> None:
    policy = PersonSafetyPolicy(PolicyConfig(hysteresis_m=0.2))

    assert evaluate(policy, 0.0, [observation(1.7)]).state == SafetyState.SLOW
    assert evaluate(policy, 0.1, [observation(1.9)]).state == SafetyState.SLOW
    assert evaluate(policy, 0.2, [observation(2.01)]).state == SafetyState.CAUTION


def test_low_confidence_detection_is_ignored() -> None:
    policy = PersonSafetyPolicy(PolicyConfig(minimum_confidence=0.5))

    result = evaluate(policy, 0.0, [observation(0.2, confidence=0.49)])

    assert result.state == SafetyState.CLEAR
    assert result.person_count == 0


def test_nearest_valid_person_controls_state() -> None:
    policy = PersonSafetyPolicy(PolicyConfig())

    result = evaluate(
        policy,
        0.0,
        [observation(3.0), observation(0.8), observation(1.5)],
    )

    assert result.state == SafetyState.STOP
    assert result.person_count == 3
    assert result.nearest_distance_m == pytest.approx(0.8)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"stop_distance_m": 2.0, "slow_distance_m": 1.0},
        {"minimum_confidence": 1.1},
        {"detection_timeout_sec": 0.0},
        {"clear_hold_sec": -1.0},
        {"hysteresis_m": -0.1},
    ],
)
def test_invalid_configuration_is_rejected(kwargs) -> None:
    with pytest.raises(ValueError):
        PolicyConfig(**kwargs)
