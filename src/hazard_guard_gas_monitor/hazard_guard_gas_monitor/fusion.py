from __future__ import annotations

from dataclasses import dataclass


SEVERITY = {"normal": 0, "watch": 1, "warning": 2, "critical": 3}


@dataclass(frozen=True)
class FusionDecision:
    level: str
    reason: str
    title: str


def thermal_identity(source: str, temperature_c: float) -> tuple[str, str, str]:
    """Read equipment and severity from real analyzer or simulator detections."""

    parts = str(source).split(":")
    if len(parts) >= 3 and parts[0] == "thermal_trend":
        return parts[1], parts[2], parts[3] if len(parts) >= 4 else ""
    if len(parts) >= 2 and parts[0] == "gazebo":
        status = (
            "critical"
            if temperature_c >= 80.0
            else "warning"
            if temperature_c >= 60.0
            else "watch"
            if temperature_c >= 45.0
            else "normal"
        )
        return parts[1], status, "simulated_temperature_screening"
    return "", "normal", "unsupported_thermal_source"


def fuse_risk(
    *,
    voc_abnormal: bool,
    co_warning: bool,
    co_critical: bool,
    localized_voc: bool,
    thermal_status: str,
    same_zone: bool,
) -> FusionDecision:
    """Combine independent gas and thermal evidence without downgrading either."""

    thermal_level = (
        thermal_status if thermal_status in SEVERITY else "normal"
    )
    thermal_watch = same_zone and SEVERITY[thermal_level] >= SEVERITY["watch"]
    thermal_warning = (
        same_zone and SEVERITY[thermal_level] >= SEVERITY["warning"]
    )

    if co_critical:
        return FusionDecision(
            "critical", "co_absolute_critical", "연소 위험 가스 긴급"
        )
    if same_zone and thermal_level == "critical":
        return FusionDecision(
            "critical", "thermal_absolute_critical", "고온·가스 위험 긴급"
        )
    if voc_abnormal and co_warning and thermal_watch:
        return FusionDecision(
            "critical",
            "voc_co_thermal_corroborated",
            "배터리 열폭주 위험 긴급",
        )
    if co_warning and thermal_warning:
        return FusionDecision(
            "critical",
            "co_and_thermal_warning_corroborated",
            "연소·고온 복합 위험 긴급",
        )
    if co_warning:
        return FusionDecision(
            "warning", "co_warning", "CO 동반 가스 이상 경고"
        )
    if voc_abnormal and thermal_watch:
        return FusionDecision(
            "warning",
            "voc_and_thermal_watch_corroborated",
            "가스·열화상 복합 이상 경고",
        )
    if thermal_warning:
        return FusionDecision(
            "warning", "thermal_warning", "동일 구역 고온 경고"
        )
    if localized_voc:
        return FusionDecision(
            "warning",
            "persistent_voc_across_locations",
            "다지점 VOC 지속 이상 경고",
        )
    if voc_abnormal:
        return FusionDecision(
            "watch", "voc_early_watch", "VOC 급상승 관찰"
        )
    if thermal_watch:
        return FusionDecision(
            "watch", "thermal_watch", "동일 구역 열화상 관찰"
        )
    return FusionDecision("normal", "all_signals_normal", "복합 위험 정상")
