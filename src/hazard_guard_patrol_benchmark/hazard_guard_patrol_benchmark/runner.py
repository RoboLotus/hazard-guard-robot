from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
import subprocess
import sys

from .environment import load_world_assets, resolve_environment_root
from .report import default_storage_root


def _run_patrol_script(
    script_path: str,
    mission_id: str,
    environment: dict[str, str],
) -> tuple[int, bool, str]:
    """Run one patrol while preserving logs and validating the Action result.

    ``ros2 action send_goal`` may exit with code 0 even when the server returns an
    application-level failure.  The generated Simulation_env scripts print the
    RunPatrol result, so a benchmark run is accepted only when both the process
    and the Action result report success.
    """
    process = subprocess.Popen(
        ["bash", script_path, mission_id],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    output_lines: list[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        output_lines.append(line)
        print(line, end="", flush=True)
    exit_code = process.wait()
    output = "".join(output_lines)
    action_succeeded = "success: true" in output.lower()
    reason = "completed"
    if exit_code != 0:
        reason = f"patrol script exit code {exit_code}"
    elif not action_succeeded:
        reason = "RunPatrol Action did not report success: true"
    return exit_code, action_succeeded, reason


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
        exit_code, action_succeeded, reason = _run_patrol_script(
            str(assets.patrol_script_path),
            mission_id,
            environment,
        )
        result = {
            "mission_id": mission_id,
            "exit_code": exit_code,
            "action_succeeded": action_succeeded,
            "reason": reason,
        }
        results.append(result)
        if (exit_code != 0 or not action_succeeded) and not args.continue_on_failure:
            break
    batch_root = default_storage_root() / "batches"
    batch_root.mkdir(parents=True, exist_ok=True)
    output = batch_root / f"{batch_id}.json"
    output.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Batch manifest: {output}")
    if any(
        item["exit_code"] != 0 or not item["action_succeeded"]
        for item in results
    ):
        sys.exit(1)


if __name__ == "__main__":
    main()
