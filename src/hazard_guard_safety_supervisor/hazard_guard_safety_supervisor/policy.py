"""Pure person-safety state policy.

This module deliberately has no ROS imports so the transition rules can be
tested on a development machine without a sourced ROS installation.
"""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Iterable, Optional, Sequence


class SafetyState(IntEnum):
    """Wire-compatible values used by ``PersonSafetyState.msg``."""

    CLEAR = 0
    CAUTION = 1
    SLOW = 2
    STOP = 3
    SENSOR_FAULT = 4


@dataclass(frozen=True)
class PersonObservation:
    """The subset of a person observation required by the safety policy."""

    confidence: float
    distance_m: float = math.nan
    distance_valid: bool = False


@dataclass(frozen=True)
class PolicyConfig:
    caution_distance_m: float = 2.5
    slow_distance_m: float = 1.8
    stop_distance_m: float = 0.9
    minimum_confidence: float = 0.4
    detection_timeout_sec: float = 0.5
    clear_hold_sec: float = 2.0
    hysteresis_m: float = 0.2

    def __post_init__(self) -> None:
        if not 0.0 < self.stop_distance_m < self.slow_distance_m < self.caution_distance_m:
            raise ValueError(
                "distance thresholds must satisfy 0 < stop < slow < caution"
            )
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be between 0 and 1")
        if self.detection_timeout_sec <= 0.0:
            raise ValueError("detection_timeout_sec must be positive")
        if self.clear_hold_sec < 0.0:
            raise ValueError("clear_hold_sec cannot be negative")
        if self.hysteresis_m < 0.0:
            raise ValueError("hysteresis_m cannot be negative")


@dataclass(frozen=True)
class PolicyResult:
    state: SafetyState
    person_count: int
    nearest_distance_m: float
    distance_valid: bool
    detector_stale: bool
    reason: str


class PersonSafetyPolicy:
    """Stateful finite-state policy with conservative release behavior."""

    def __init__(self, config: PolicyConfig) -> None:
        self.config = config
        self._state = SafetyState.CLEAR
        self._clear_candidate_since: Optional[float] = None

    @property
    def state(self) -> SafetyState:
        return self._state

    def evaluate(
        self,
        *,
        now_sec: float,
        observations: Iterable[PersonObservation],
        last_detector_update_sec: Optional[float],
        autonomous: bool,
        sensor_healthy: bool = True,
        sensor_reason: str = "",
    ) -> PolicyResult:
        observations = tuple(self._accepted_observations(observations))
        stale = self._is_stale(now_sec, last_detector_update_sec)

        if autonomous and stale:
            self._state = SafetyState.SENSOR_FAULT
            self._clear_candidate_since = None
            return self._result(
                observations,
                stale=True,
                reason="detector data is stale while autonomous driving is enabled",
            )

        if autonomous and not sensor_healthy:
            self._state = SafetyState.SENSOR_FAULT
            self._clear_candidate_since = None
            return self._result(
                observations,
                stale=False,
                reason=sensor_reason or "RGB-D sensor health is invalid",
            )

        valid_distances = tuple(
            observation.distance_m
            for observation in observations
            if self._has_valid_distance(observation)
        )
        has_invalid_distance = any(
            not self._has_valid_distance(observation) for observation in observations
        )

        if has_invalid_distance:
            # A detected person with unknown distance must not release an
            # existing restrictive state. CAUTION is the least restrictive
            # state permitted for this ambiguous observation. Unknown distance
            # must reduce motion even when the RGB-D stream itself is healthy.
            target = (
                self._state_for_distance(min(valid_distances))
                if valid_distances
                else SafetyState.SLOW
            )
            target = max(target, SafetyState.SLOW)
            if self._state != SafetyState.SENSOR_FAULT:
                target = max(target, self._state)
            self._state = target
            self._clear_candidate_since = None
            distance_reason = (
                f"; nearest valid estimate is {min(valid_distances):.2f} m"
                if valid_distances
                else ""
            )
            return self._result(
                observations,
                stale=stale,
                reason=f"at least one detected person has no distance{distance_reason}",
            )

        if valid_distances:
            nearest = min(valid_distances)
            target = self._state_for_distance(nearest)
            if target == SafetyState.CLEAR:
                return self._evaluate_clear_candidate(now_sec, observations, stale)
            self._state = target
            self._clear_candidate_since = None
            return self._result(
                observations,
                stale=stale,
                reason=f"nearest person is {nearest:.2f} m away",
            )

        return self._evaluate_clear_candidate(now_sec, observations, stale)

    def _accepted_observations(
        self, observations: Iterable[PersonObservation]
    ) -> Sequence[PersonObservation]:
        return tuple(
            observation
            for observation in observations
            if math.isfinite(observation.confidence)
            and observation.confidence >= self.config.minimum_confidence
        )

    @staticmethod
    def _has_valid_distance(observation: PersonObservation) -> bool:
        return (
            observation.distance_valid
            and math.isfinite(observation.distance_m)
            and observation.distance_m >= 0.0
        )

    def _is_stale(
        self, now_sec: float, last_detector_update_sec: Optional[float]
    ) -> bool:
        return (
            last_detector_update_sec is None
            or now_sec - last_detector_update_sec > self.config.detection_timeout_sec
        )

    def _state_for_distance(self, distance_m: float) -> SafetyState:
        hysteresis = self.config.hysteresis_m

        if distance_m <= self.config.stop_distance_m:
            nominal = SafetyState.STOP
        elif distance_m <= self.config.slow_distance_m:
            nominal = SafetyState.SLOW
        elif distance_m <= self.config.caution_distance_m:
            nominal = SafetyState.CAUTION
        else:
            nominal = SafetyState.CLEAR

        # Enter a more restrictive distance zone immediately. Hysteresis is
        # applied only while leaving a zone, never while a person approaches.
        if self._state == SafetyState.SENSOR_FAULT or nominal > self._state:
            return nominal

        if (
            self._state == SafetyState.STOP
            and distance_m <= self.config.stop_distance_m + hysteresis
        ):
            return SafetyState.STOP
        if (
            self._state == SafetyState.SLOW
            and distance_m <= self.config.slow_distance_m + hysteresis
        ):
            return SafetyState.SLOW
        if (
            self._state == SafetyState.CAUTION
            and distance_m <= self.config.caution_distance_m + hysteresis
        ):
            return SafetyState.CAUTION
        return nominal

    def _evaluate_clear_candidate(
        self,
        now_sec: float,
        observations: Sequence[PersonObservation],
        stale: bool,
    ) -> PolicyResult:
        if self._state == SafetyState.CLEAR:
            self._clear_candidate_since = None
            return self._result(
                observations,
                stale=stale,
                reason="no person is inside the safety zones",
            )

        if self._clear_candidate_since is None:
            self._clear_candidate_since = now_sec

        held_for = max(0.0, now_sec - self._clear_candidate_since)
        if held_for + 1e-9 >= self.config.clear_hold_sec:
            self._state = SafetyState.CLEAR
            self._clear_candidate_since = None
            reason = "clear condition held long enough to resume normal operation"
        else:
            remaining = self.config.clear_hold_sec - held_for
            reason = f"waiting {remaining:.2f} s before clearing the safety state"

        return self._result(observations, stale=stale, reason=reason)

    def _result(
        self,
        observations: Sequence[PersonObservation],
        *,
        stale: bool,
        reason: str,
    ) -> PolicyResult:
        distances = tuple(
            observation.distance_m
            for observation in observations
            if self._has_valid_distance(observation)
        )
        return PolicyResult(
            state=self._state,
            person_count=len(observations),
            nearest_distance_m=min(distances) if distances else math.nan,
            distance_valid=bool(distances),
            detector_stale=stale,
            reason=reason,
        )
