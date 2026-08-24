from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
import subprocess
import sys

from .environment import load_world_assets, resolve_environment_root
from .report import default_storage_root


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an existing Simulation_env patrol route repeatedly."
    )
    parser.add_argument("--world-id", default="real_factory")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--dwell-seconds", type=float, default=3.0)
    parser.add_argument("--simulation-env-path", default="")
    parser.add_argument("--continue-on-failure", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = _arguments()
    if args.runs < 1:
        raise SystemExit("--runs는 1 이상이어야 합니다")
    root = resolve_environment_root(args.simulation_env_path)
    assets = load_world_assets(root, args.world_id)
    if assets.patrol_script_path is None:
        raise SystemExit(
            f"'{args.world_id}'의 반복 실행 가능한 순찰 스크립트가 없습니다"
        )
    environment = os.environ.copy()
    environment["PATROL_DWELL_SECONDS"] = str(max(0.0, args.dwell_seconds))
    batch_id = datetime.now(timezone.utc).strftime("benchmark-%Y%m%dT%H%M%SZ")
    results: list[dict[str, object]] = []
    manifest: dict[str, object] = {
        "batch_id": batch_id,
        "world_id": args.world_id,
        "runs": args.runs,
        "dwell_seconds": args.dwell_seconds,
        "results": results,
    }
    for index in range(1, args.runs + 1):
        mission_id = f"{batch_id}-run-{index:03d}"
        print(f"[{index}/{args.runs}] {mission_id}", flush=True)
        completed = subprocess.run(
            ["bash", str(assets.patrol_script_path), mission_id],
            env=environment,
            check=False,
        )
        result = {"mission_id": mission_id, "exit_code": completed.returncode}
        results.append(result)
        if completed.returncode != 0 and not args.continue_on_failure:
            break
    batch_root = default_storage_root() / "batches"
    batch_root.mkdir(parents=True, exist_ok=True)
    output = batch_root / f"{batch_id}.json"
    output.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Batch manifest: {output}")
    if any(item["exit_code"] != 0 for item in results):
        sys.exit(1)


if __name__ == "__main__":
    main()
