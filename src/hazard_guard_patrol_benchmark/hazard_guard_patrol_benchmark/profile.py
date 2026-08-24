from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from math import ceil
from typing import Any, Iterable


REQUIRED_SIM_RATES_HZ = {
    "scan": 7.0,
    # Image and PointCloud2 bridges can legitimately drop transport frames
    # under load even though the Gazebo sensors themselves keep their update
    # rate. The health gate detects stalled streams; the exact achieved rates
    # remain comparison metrics rather than pass/fail targets.
    "rgb": 5.0,
    "depth": 5.0,
    # PointCloud2 transport is much heavier than the depth image and the ROS-GZ
    # bridge intentionally does not promise the camera's full update rate.
    "points": 0.2,
    "thermal": 5.5,
}


def percentile(values: Iterable[float], probability: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    index = max(0, min(len(ordered) - 1, ceil(probability * len(ordered)) - 1))
    return round(ordered[index], 4)


def sample_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return {"count": 0, "mean": None, "median": None, "p95": None}
    middle = len(ordered) // 2
    median = (
        ordered[middle]
        if len(ordered) % 2
        else (ordered[middle - 1] + ordered[middle]) / 2.0
    )
    return {
        "count": len(ordered),
        "mean": round(sum(ordered) / len(ordered), 4),
        "median": round(median, 4),
        "p95": percentile(ordered, 0.95),
    }


def probe_is_healthy(document: dict[str, Any]) -> bool:
    if not document.get("gazebo_alive") or float(document.get("sim_seconds") or 0) <= 0:
        return False
    rates = document.get("topic_rates_sim_hz") or {}
    return all(
        float(rates.get(topic) or 0) >= minimum
        for topic, minimum in REQUIRED_SIM_RATES_HZ.items()
    )


def probe_health_failures(document: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if not document.get("gazebo_alive"):
        failures.append("gazebo_not_alive")
    if float(document.get("sim_seconds") or 0) <= 0:
        failures.append("simulation_time_not_advancing")
    rates = document.get("topic_rates_sim_hz") or {}
    for topic, minimum in REQUIRED_SIM_RATES_HZ.items():
        actual = float(rates.get(topic) or 0)
        if actual < minimum:
            failures.append(f"{topic}_rate_below_{minimum:g}hz:{actual:.4f}")
    return failures


@dataclass(frozen=True)
class ProfileChoice:
    automated: str | None
    visual: str | None


def aggregate_profiles(documents: Iterable[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for document in documents:
        grouped[str(document["profile_id"])].append(document)

    profiles: dict[str, Any] = {}
    for profile_id, runs in sorted(grouped.items()):
        healthy_runs = [run for run in runs if probe_is_healthy(run)]
        failed_runs = [
            {
                "run": index + 1,
                "failures": probe_health_failures(run),
            }
            for index, run in enumerate(runs)
            if not probe_is_healthy(run)
        ]
        profiles[profile_id] = {
            "run_count": len(runs),
            "healthy_count": len(healthy_runs),
            "all_healthy": len(healthy_runs) == len(runs),
            "gui": bool(runs[0].get("gui")),
            "gpu_enabled": bool(runs[0].get("gpu_enabled")),
            "render_engine": runs[0].get("render_engine"),
            "failed_runs": failed_runs,
            "real_time_factor": sample_summary(
                float(run["real_time_factor"]) for run in runs
            ),
            "process_cpu_percent": sample_summary(
                float(run["process_cpu_percent"]["median"])
                for run in runs
                if run.get("process_cpu_percent", {}).get("median") is not None
            ),
            "process_rss_mib": sample_summary(
                float(run["process_rss_mib"]["median"])
                for run in runs
                if run.get("process_rss_mib", {}).get("median") is not None
            ),
            "gpu_utilization_percent": sample_summary(
                float(run["gpu_utilization_percent"]["median"])
                for run in runs
                if run.get("gpu_utilization_percent", {}).get("median") is not None
            ),
            "topic_rates_sim_hz": {
                topic: sample_summary(
                    float(run.get("topic_rates_sim_hz", {}).get(topic, 0))
                    for run in runs
                )
                for topic in REQUIRED_SIM_RATES_HZ
            },
        }

    def select(gui: bool) -> str | None:
        candidates = [
            (profile_id, profile)
            for profile_id, profile in profiles.items()
            if profile["gui"] is gui and profile["all_healthy"]
        ]
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda item: (
                float(item[1]["real_time_factor"]["median"] or 0),
                -float(item[1]["process_cpu_percent"]["median"] or 0),
            ),
        )[0]

    choice = ProfileChoice(automated=select(False), visual=select(True))
    return {
        "schema_version": 1,
        "profiles": profiles,
        "recommended": {
            "automated": choice.automated,
            "visual": choice.visual,
        },
    }
