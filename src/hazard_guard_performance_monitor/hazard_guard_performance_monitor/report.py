from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
from typing import Any

from .statistics import summarize_values


SAFE_ID = re.compile(r"[^A-Za-z0-9_.-]+")


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def safe_report_id(value: str) -> str:
    cleaned = SAFE_ID.sub("-", value.strip()).strip("-.")
    return cleaned[:80] or "mission"


def default_storage_root() -> Path:
    configured = os.getenv("HAZARD_GUARD_PERFORMANCE_DIR")
    if configured:
        return Path(configured).expanduser().resolve()
    return (Path.home() / ".local/share/hazard-guard/performance").resolve()


def _metric(samples: list[dict[str, Any]], *path: str) -> dict[str, Any]:
    values: list[float] = []
    for sample in samples:
        item: Any = sample
        for key in path:
            if not isinstance(item, dict):
                item = None
                break
            item = item.get(key)
        if isinstance(item, (int, float)) and not isinstance(item, bool):
            values.append(float(item))
    return summarize_values(values)


def _process_summaries(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    labels = sorted(
        {
            str(process.get("label"))
            for sample in samples
            for process in sample.get("processes", [])
            if process.get("label")
        }
    )
    result: list[dict[str, Any]] = []
    for label in labels:
        per_sample = []
        for sample in samples:
            matching = [
                process
                for process in sample.get("processes", [])
                if process.get("label") == label
            ]
            if not matching:
                continue
            per_sample.append(
                {
                    name: sum(
                        float(item[name])
                        for item in matching
                        if isinstance(item.get(name), (int, float))
                    )
                    for name in ("cpu_core_percent", "rss_mb", "threads")
                }
            )
        result.append(
            {
                "label": label,
                "cpu_core_percent": summarize_values(
                    float(item["cpu_core_percent"])
                    for item in per_sample
                    if isinstance(item.get("cpu_core_percent"), (int, float))
                ),
                "rss_mb": summarize_values(
                    float(item["rss_mb"])
                    for item in per_sample
                    if isinstance(item.get("rss_mb"), (int, float))
                ),
                "threads": summarize_values(
                    float(item["threads"])
                    for item in per_sample
                    if isinstance(item.get("threads"), (int, float))
                ),
            }
        )
    return result


def summarize_session(
    metadata: dict[str, Any],
    samples: list[dict[str, Any]],
    result_status: str,
    finished_at: str,
) -> dict[str, Any]:
    phases: dict[str, list[dict[str, Any]]] = {}
    core_names: set[str] = set()
    for sample in samples:
        phases.setdefault(
            str(sample.get("phase") or "unknown"), []
        ).append(sample)
        core_names.update((sample.get("cpu") or {}).get("cores", {}).keys())

    started_at = metadata["started_at"]
    duration_sec = 0.0
    if samples:
        duration_sec = max(0.0, float(samples[-1]["elapsed_sec"]))
    return {
        "schema_version": 1,
        "id": metadata["id"],
        "name": metadata["name"],
        "mission_id": metadata.get("mission_id"),
        "mission_name": metadata.get("mission_name"),
        "status": result_status,
        "started_at": started_at,
        "finished_at": finished_at,
        "duration_sec": round(duration_sec, 3),
        "sample_count": len(samples),
        "platform": metadata.get("platform", {}),
        "system": {
            "cpu_percent": _metric(samples, "cpu", "total_percent"),
            "ram_percent": _metric(samples, "memory", "used_percent"),
            "ram_used_mb": _metric(samples, "memory", "used_mb"),
            "swap_used_mb": _metric(samples, "memory", "swap_used_mb"),
            "gpu_percent": _metric(samples, "jetson", "gpu_percent"),
            "cpu_temperature_c": _metric(
                samples, "jetson", "temperatures_c", "cpu"
            ),
            "gpu_temperature_c": _metric(
                samples, "jetson", "temperatures_c", "gpu"
            ),
            "input_power_mw": _metric(samples, "jetson", "power_mw", "vdd_in"),
        },
        "cores": {
            name: _metric(samples, "cpu", "cores", name)
            for name in sorted(core_names)
        },
        "processes": _process_summaries(samples),
        "phases": {
            name: {
                "sample_count": len(items),
                "cpu_percent": _metric(items, "cpu", "total_percent"),
                "ram_percent": _metric(items, "memory", "used_percent"),
                "gpu_percent": _metric(items, "jetson", "gpu_percent"),
            }
            for name, items in sorted(phases.items())
        },
    }


def _atomic_json(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _write_csv(path: Path, summary: dict[str, Any]) -> None:
    metric_names = ("count", "mean", "median", "p95", "max", "stddev")
    rows: list[list[Any]] = [["category", "name", *metric_names]]
    for name, metric in summary["system"].items():
        rows.append(
            ["system", name, *[metric.get(key) for key in metric_names]]
        )
    for name, metric in summary["cores"].items():
        rows.append(
            ["core", name, *[metric.get(key) for key in metric_names]]
        )
    for process in summary["processes"]:
        for name in ("cpu_core_percent", "rss_mb", "threads"):
            metric = process[name]
            rows.append(
                [
                    f"process:{process['label']}",
                    name,
                    *[metric.get(key) for key in metric_names],
                ]
            )
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        csv.writer(stream).writerows(rows)


def _format_metric(metric: dict[str, Any], unit: str) -> str:
    if not metric or metric.get("count", 0) == 0:
        return "측정 없음"
    return (
        f"평균 {metric['mean']}{unit} / 중앙값 {metric['median']}{unit} / "
        f"P95 {metric['p95']}{unit} / 최대 {metric['max']}{unit}"
    )


def _write_markdown(path: Path, summary: dict[str, Any]) -> None:
    system = summary["system"]
    lines = [
        f"# {summary['name']}",
        "",
        f"- 임무 ID: `{summary.get('mission_id') or '-'}`",
        f"- 결과: `{summary['status']}`",
        f"- 측정 시간: {summary['duration_sec']:.1f}초",
        f"- 유효 샘플: {summary['sample_count']}개",
        "",
        "## 시스템 요약",
        "",
        f"- CPU: {_format_metric(system['cpu_percent'], '%')}",
        f"- GPU: {_format_metric(system['gpu_percent'], '%')}",
        f"- RAM: {_format_metric(system['ram_percent'], '%')}",
        "",
        "## 프로세스별 요약",
        "",
        "| 프로세스 | CPU 평균 | CPU P95 | RAM 평균 | RAM P95 |",
        "|---|---:|---:|---:|---:|",
    ]
    for process in summary["processes"]:
        cpu = process["cpu_core_percent"]
        ram = process["rss_mb"]
        lines.append(
            f"| {process['label']} | {cpu.get('mean') or '-'}% | "
            f"{cpu.get('p95') or '-'}% | {ram.get('mean') or '-'} MB | "
            f"{ram.get('p95') or '-'} MB |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class PerformanceSession:
    def __init__(
        self,
        storage_root: Path,
        mission: dict[str, Any],
        platform: dict[str, Any],
        started_at: datetime | None = None,
    ) -> None:
        started = started_at or utc_now()
        mission_id = safe_report_id(
            str(mission.get("mission_id") or "mission")
        )
        base_report_id = f"{started.strftime('%Y%m%dT%H%M%SZ')}-{mission_id}"
        report_id = base_report_id
        date_directory = storage_root / started.strftime("%Y-%m-%d")
        suffix = 1
        while True:
            self.directory = date_directory / report_id
            try:
                self.directory.mkdir(parents=True, exist_ok=False)
                break
            except FileExistsError:
                suffix += 1
                report_id = f"{base_report_id}-{suffix}"
        self.metadata = {
            "schema_version": 1,
            "id": report_id,
            "name": str(mission.get("name") or mission_id),
            "mission_id": mission.get("mission_id"),
            "mission_name": mission.get("name"),
            "started_at": started.isoformat(),
            "platform": platform,
        }
        self.sample_count = 0
        self._stream = (self.directory / "samples.jsonl").open(
            "a", encoding="utf-8", buffering=1
        )
        _atomic_json(self.directory / "metadata.json", self.metadata)

    def append(self, sample: dict[str, Any]) -> None:
        self.sample_count += 1
        payload = json.dumps(
            sample,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self._stream.write(payload + "\n")
        self._stream.flush()
        _atomic_json(
            self.directory / "active.json",
            {
                "id": self.metadata["id"],
                "name": self.metadata["name"],
                "mission_id": self.metadata.get("mission_id"),
                "started_at": self.metadata["started_at"],
                "updated_at": sample["timestamp"],
                "sample_count": self.sample_count,
                "phase": sample.get("phase"),
                "latest": sample,
            },
        )

    def finalize(
        self,
        status: str,
        finished_at: datetime | None = None,
    ) -> dict[str, Any]:
        self._stream.close()
        finished = finished_at or utc_now()
        samples = []
        with (self.directory / "samples.jsonl").open(
            encoding="utf-8"
        ) as stream:
            for line in stream:
                try:
                    sample = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(sample, dict):
                    samples.append(sample)
        summary = summarize_session(
            self.metadata,
            samples,
            status,
            finished.isoformat(),
        )
        _atomic_json(self.directory / "summary.json", summary)
        _write_csv(self.directory / "process-summary.csv", summary)
        _write_markdown(self.directory / "report.md", summary)
        (self.directory / "active.json").unlink(missing_ok=True)
        return summary

    def discard(self) -> None:
        if not self._stream.closed:
            self._stream.close()
        shutil.rmtree(self.directory, ignore_errors=True)
