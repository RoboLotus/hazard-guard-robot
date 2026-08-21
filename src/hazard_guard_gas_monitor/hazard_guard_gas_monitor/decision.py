from __future__ import annotations

from dataclasses import dataclass

from .model import DecisionConfig, GasVector


@dataclass(frozen=True)
class GasDecision:
    state: str
    level: str
    reason: str
    navigation_pause: bool
    fan_on: bool
    changed: bool


class GasDecisionEngine:
    """VOC raises a watch; CO raises confidence/severity but never cancels VOC."""

    PAUSED_STATES = {"voc_watch", "warning", "critical"}
    FAN_STATES = PAUSED_STATES | {"investigating"}

    def __init__(
        self,
        ambient: GasVector,
        config: DecisionConfig,
        baseline_ready: bool = True,
    ) -> None:
        self.ambient = ambient
        self.config = config
        self.baseline_ready = baseline_ready
        self.state = "warming_up"
        self._voc_samples = 0
        self._clear_samples = 0
        self._watch_started_sec: float | None = None

    def set_ambient(self, ambient: GasVector) -> None:
        self.ambient = ambient
        self.baseline_ready = True

    def voc_abnormal(self, reading: GasVector) -> bool:
        return (
            reading.voc_index >= self.config.voc_absolute_watch_index
            or (
                self.baseline_ready
                and reading.voc_index - self.ambient.voc_index
                >= self.config.voc_watch_delta
            )
        )

    def voc_cleared(self, reading: GasVector) -> bool:
        if not self.baseline_ready:
            return reading.voc_index < self.config.voc_absolute_watch_index
        return (
            reading.voc_index < self.config.voc_absolute_watch_index
            and reading.voc_index - self.ambient.voc_index
            <= self.config.voc_clear_delta
        )

    def co_warning(self, reading: GasVector) -> bool:
        return (
            reading.co_ppm >= self.config.co_absolute_warning_ppm
            or (
                self.baseline_ready
                and reading.co_ppm - self.ambient.co_ppm
                >= self.config.co_warning_delta_ppm
            )
        )

    def co_critical(self, reading: GasVector) -> bool:
        return reading.co_ppm >= self.config.co_absolute_critical_ppm

    def update(self, reading: GasVector, elapsed_sec: float, warmed_up: bool) -> GasDecision:
        previous = self.state

        if not warmed_up:
            next_state, reason = "warming_up", "sensor_warmup"
        elif self.co_critical(reading):
            next_state, reason = "critical", "co_absolute_critical"
        elif self.co_warning(reading):
            reason = (
                "co_absolute_warning"
                if reading.co_ppm >= self.config.co_absolute_warning_ppm
                else "co_baseline_excursion"
            )
            next_state = "warning"
        else:
            if self.voc_abnormal(reading):
                self._voc_samples += 1
                self._clear_samples = 0
            elif self.voc_cleared(reading):
                self._clear_samples += 1
                self._voc_samples = 0
            else:
                self._clear_samples = 0

            if self._voc_samples >= self.config.voc_persistent_samples:
                if self._watch_started_sec is None:
                    self._watch_started_sec = elapsed_sec
                age = elapsed_sec - self._watch_started_sec
                if age >= self.config.investigation_delay_sec:
                    next_state = "investigating"
                    reason = "persistent_voc_requires_local_search"
                else:
                    next_state, reason = "voc_watch", "voc_rise_requires_stationary_recheck"
            elif (
                previous in self.PAUSED_STATES
                and self._clear_samples < self.config.clear_samples
            ):
                next_state, reason = previous, "gas_clearance_hold"
            else:
                next_state, reason = "normal", "gas_levels_normal"
                self._watch_started_sec = None

        self.state = next_state
        levels = {
            "warming_up": "info",
            "normal": "info",
            "voc_watch": "watch",
            "investigating": "watch",
            "warning": "warning",
            "critical": "critical",
        }
        return GasDecision(
            state=next_state,
            level=levels[next_state],
            reason=reason,
            navigation_pause=next_state in self.PAUSED_STATES,
            fan_on=next_state in self.FAN_STATES,
            changed=next_state != previous,
        )
