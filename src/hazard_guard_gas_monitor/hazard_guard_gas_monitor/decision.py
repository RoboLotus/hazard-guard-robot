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

    def __init__(self, ambient: GasVector, config: DecisionConfig) -> None:
        self.ambient = ambient
        self.config = config
        self.state = "warming_up"
        self._voc_samples = 0
        self._clear_samples = 0
        self._watch_started_sec: float | None = None

    def update(self, reading: GasVector, elapsed_sec: float, warmed_up: bool) -> GasDecision:
        previous = self.state
        voc_delta = reading.voc_index - self.ambient.voc_index
        co_delta = reading.co_ppm - self.ambient.co_ppm

        if not warmed_up:
            next_state, reason = "warming_up", "sensor_warmup"
        elif co_delta >= self.config.co_critical_delta_ppm:
            next_state, reason = "critical", "co_critical_with_gas_plume"
        elif co_delta >= self.config.co_warning_delta_ppm:
            next_state, reason = "warning", "co_confirms_combustion_risk"
        else:
            if voc_delta >= self.config.voc_watch_delta:
                self._voc_samples += 1
                self._clear_samples = 0
            elif voc_delta <= self.config.voc_clear_delta:
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
