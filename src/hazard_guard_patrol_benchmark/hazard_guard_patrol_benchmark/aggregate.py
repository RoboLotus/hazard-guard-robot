from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from .metrics import summarize
from .report import default_storage_root


METRICS = {
    "simulation_sec": ("time", "simulation_sec"),
    "wall_sec": ("time", "wall_sec"),
    "real_time_factor": ("time", "real_time_factor"),
    "actual_distance_m": ("trajectory", "actual_distance_m"),
    "path_efficiency_percent": ("trajectory", "path_efficiency_percent"),
    "observed_area_m2": ("coverage", "observed_area_m2"),
    "space_coverage_percent": ("coverage", "coverage_percent"),
    "coverage_rate_m2_per_min": ("coverage", "coverage_rate_m2_per_min"),
    "time_to_90_percent_sec": ("coverage", "time_to_90_percent_sec"),
    "coverage_revisit_percent": ("coverage", "revisit_percent"),
    "waypoint_completion_percent": ("waypoints", "completion_percent"),
    "waypoint_position_error_p95_m": (
        "segments",
        "arrival_position_error_m",
        "p95",
    ),
    "waypoint_yaw_error_p95_deg": (
        "segments",
        "arrival_yaw_error_deg",
        "p95",
    ),
    "thermal_coverage_percent": ("thermal", "coverage_percent"),
    "thermal_precision": ("thermal", "precision"),
    "first_thermal_detection_sec": ("thermal", "first_expected_detection_sec"),
    "collision_count": ("safety", "collision_count"),
    "near_miss_count": ("safety", "near_miss_count"),
    "minimum_clearance_m": ("safety", "minimum_clearance_m"),
    "recovery_count": ("safety", "recovery_count"),
    "global_plan_update_count": ("safety", "global_plan_update_count"),
}


def _value(document: dict[str, Any], path: tuple[str, ...]) -> float | None:
    value: Any = document
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return None


def aggregate_summaries(paths: list[Path]) -> dict[str, Any]:
    documents = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    success_count = sum(
        1 for document in documents if document.get("status") == "completed"
    )
    return {
        "schema_version": 1,
        "run_count": len(documents),
        "success_count": success_count,
        "success_rate_percent": round(
            success_count / len(documents) * 100.0 if documents else 0.0,
            3,
        ),
        "metrics": {
            name: summarize(
                value
                for document in documents
                if (value := _value(document, path)) is not None
            )
            for name, path in METRICS.items()
        },
        "report_ids": [document.get("id") for document in documents],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate benchmark reports.")
    parser.add_argument("--storage-path", default="")
    parser.add_argument("--world-id", default="real_factory")
    parser.add_argument("--latest", type=int, default=10)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    root = (
        Path(args.storage_path).expanduser().resolve()
        if args.storage_path
        else default_storage_root()
    )
    candidates = sorted(root.glob("*/*/summary.json"), reverse=True)
    selected: list[Path] = []
    for path in candidates:
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("world_id") == args.world_id:
            selected.append(path)
        if len(selected) >= max(1, args.latest):
            break
    if not selected:
        raise SystemExit(f"'{args.world_id}' 벤치마크 결과가 없습니다: {root}")
    summary = aggregate_summaries(selected)
    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else (
            root
            / "aggregates"
            / (
                f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-"
                f"{args.world_id}-latest-{len(selected)}"
            )
        )
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    with output.with_suffix(".csv").open(
        "w", encoding="utf-8-sig", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "count", "mean", "median", "p95", "min", "max"])
        for name, metric in summary["metrics"].items():
            writer.writerow(
                [name, *[metric[key] for key in ("count", "mean", "median", "p95", "min", "max")]]
            )
    markdown = [
        f"# {args.world_id} 순찰 벤치마크 요약",
        "",
        f"- 실행: {summary['run_count']}회",
        f"- 성공: {summary['success_count']}회",
        "",
        "| 지표 | 평균 | 중앙값 | P95 | 최소 | 최대 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, metric in summary["metrics"].items():
        markdown.append(
            f"| {name} | {metric['mean']} | {metric['median']} | "
            f"{metric['p95']} | {metric['min']} | {metric['max']} |"
        )
    output.with_suffix(".md").write_text(
        "\n".join(markdown) + "\n", encoding="utf-8"
    )
    print(output.with_suffix(".json"))


if __name__ == "__main__":
    main()
