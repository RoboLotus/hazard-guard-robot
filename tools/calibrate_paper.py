"""Paper-based RGB/thermal calibration - entry point.

The legacy method in tools/calibrate_thermal_rgb.py is untouched and still the
reference. This runs the other one:

    python3 tools/paper_calib/target.py                  # build the board once
    python3 tools/calibrate_paper.py capture             # collect views in Gazebo
    python3 tools/calibrate_paper.py solve               # Mode A, unconstrained
"""
from __future__ import annotations

import argparse
import json
import runpy
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "runtime" / "calibration"
sys.path.insert(0, str(ROOT / "tools"))

from paper_calib import compare as cmp  # noqa: E402
from paper_calib import optimize as opt  # noqa: E402


def load(path: Path) -> dict:
    raw = np.load(path, allow_pickle=False)
    views = len(raw["rgb_matched"])
    # Stored once because every view of one capture shares a board, but carried
    # per view from here on: two captures made with different board sizes have
    # to be usable together, and the corner count is the same either way.
    object_points = np.broadcast_to(
        raw["object_points"].astype(np.float64),
        (views,) + raw["object_points"].shape).copy()
    return {
        "views": views,
        "labels": [str(v) for v in raw["labels"]],
        "tir_how": [str(v) for v in raw["tir_how"]],
        "rgb_corners": raw["rgb_corners"].astype(np.float64),
        "rgb_matched": raw["rgb_matched"].astype(np.float64),
        "tir_matched": raw["tir_matched"].astype(np.float64),
        "object_points": object_points,
        "object_matched": raw["object_matched"].astype(np.float64),
        "rgb_k": raw["rgb_k"].reshape(3, 3),
        "thermal_k": raw["thermal_k"].reshape(3, 3),
        "rgb_size": tuple(int(v) for v in raw["rgb_size"]),
        "thermal_size": tuple(int(v) for v in raw["thermal_size"]),
        "attempted": int(raw["poses_attempted"]) if "poses_attempted" in raw else None,
    }


def merge(datasets: list) -> dict:
    """Stack captures into one problem.

    Range is what separates translation from rotation, and no single board
    covers a wide range: a board small enough to come to 0.5 m has thermal
    squares too small to read at 1.7 m. Two boards can, so long as the
    optimiser is told which one each view was looking at - which is what the
    per-view object points are for.
    """
    first = datasets[0]
    for other in datasets[1:]:
        if not np.allclose(first["rgb_k"], other["rgb_k"]) or \
                not np.allclose(first["thermal_k"], other["thermal_k"]):
            raise RuntimeError("카메라 내부파라미터가 다른 데이터셋은 합칠 수 없습니다")
        if first["object_points"].shape[1] != other["object_points"].shape[1]:
            raise RuntimeError("코너 개수가 다른 데이터셋은 합칠 수 없습니다")
    merged = dict(first)
    merged["views"] = sum(d["views"] for d in datasets)
    for key in ("labels", "tir_how"):
        merged[key] = [v for d in datasets for v in d[key]]
    for key in ("rgb_corners", "rgb_matched", "tir_matched",
                "object_points", "object_matched"):
        merged[key] = np.concatenate([d[key] for d in datasets])
    merged["attempted"] = sum(d["attempted"] or d["views"] for d in datasets)
    return merged


def load_all(paths) -> dict:
    datasets = [load(Path(p)) for p in paths]
    return datasets[0] if len(datasets) == 1 else merge(datasets)


def solve(arguments) -> int:
    data = load_all(arguments.views)
    print(f"뷰 {data['views']} 개, RGB {data['rgb_size']}, "
          f"열화상 {data['thermal_size']}")
    print(f"뷰당 대응점 {data['tir_matched'].shape[1]} 개, "
          f"RGB 코너 {data['rgb_corners'].shape[1]} 개")
    ladder = {}
    for how in data["tir_how"]:
        ladder[how] = ladder.get(how, 0) + 1
    print(f"열화상 검출 단계 {ladder}")

    print("\nMode A - 6DoF 무구속")
    params, report = opt.solve(data, init=arguments.init)

    translation = np.array(report["translation_mm"])
    rpy = np.array(report["rpy_deg"])
    translation_error = translation - opt.GROUND_TRUTH_MM
    rotation_error = rpy - opt.GROUND_TRUTH_RPY_DEG

    print("\n" + "=" * 46)
    print("Ground Truth (Gazebo 렌더링 기준, 최적화에는 미입력)")
    print(f"  T   = [{opt.GROUND_TRUTH_MM[0]:7.2f} {opt.GROUND_TRUTH_MM[1]:7.2f} "
          f"{opt.GROUND_TRUTH_MM[2]:7.2f}] mm")
    print(f"  RPY = [{opt.GROUND_TRUTH_RPY_DEG[0]:7.3f} "
          f"{opt.GROUND_TRUTH_RPY_DEG[1]:7.3f} {opt.GROUND_TRUTH_RPY_DEG[2]:7.3f}] deg")
    print("\nMode A 추정")
    print(f"  T   = [{translation[0]:7.2f} {translation[1]:7.2f} {translation[2]:7.2f}] mm")
    print(f"  RPY = [{rpy[0]:7.3f} {rpy[1]:7.3f} {rpy[2]:7.3f}] deg")
    print("\n오차")
    print(f"  tx {translation_error[0]:+7.2f}  ty {translation_error[1]:+7.2f}  "
          f"tz {translation_error[2]:+7.2f} mm   (크기 "
          f"{np.linalg.norm(translation_error):.2f} mm)")
    print(f"  roll {rotation_error[0]:+7.3f}  pitch {rotation_error[1]:+7.3f}  "
          f"yaw {rotation_error[2]:+7.3f} deg")
    print("\n재투영 RMS")
    print(f"  RGB    {report['rgb_rms_px']:.4f} px")
    print(f"  열화상 {report['tir_rms_px']:.4f} px")
    print(f"  전체   {report['total_rms_px']:.4f} px")
    print("=" * 46)

    worst = np.argsort(report["per_frame_tir_px"])[::-1][:5]
    print("\n열화상 오차가 큰 프레임")
    for index in worst:
        print(f"  {data['labels'][index]:14s} {report['per_frame_tir_px'][index]:.3f} px"
              f"   ({data['tir_how'][index]})")

    print("\n파라미터 민감도 (열화상 RMS 변화)")
    base, rows = opt.sensitivity(params, data)
    strongest = max(abs(r["delta_tir_rms_px"]) for r in rows) or 1.0
    for row in rows:
        share = abs(row["delta_tir_rms_px"]) / strongest
        print(f"  {row['parameter']:6s} {row['step']:>8s}  "
              f"{row['delta_tir_rms_px']:+.4f} px   {'#' * int(round(share * 20))}")
    print(f"  (기준 열화상 RMS {base:.4f} px)")

    RESULTS.mkdir(parents=True, exist_ok=True)
    payload = {
        "mode": "A",
        "views": data["views"],
        "rgb_size": list(data["rgb_size"]),
        "thermal_size": list(data["thermal_size"]),
        "thermal_detection_ladder": ladder,
        "ground_truth_mm": opt.GROUND_TRUTH_MM.tolist(),
        "ground_truth_rpy_deg": opt.GROUND_TRUTH_RPY_DEG.tolist(),
        "translation_error_mm": translation_error.tolist(),
        "translation_error_norm_mm": float(np.linalg.norm(translation_error)),
        "rotation_error_deg": rotation_error.tolist(),
        "sensitivity": rows,
        "labels": data["labels"],
    }
    payload.update(report)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = RESULTS / f"paper_modeA_{stamp}.json"
    path.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"\n저장  {path.relative_to(ROOT)}")
    return 0


def compare(arguments) -> int:
    names, results = [], []
    for index, group in enumerate(arguments.views):
        data = load_all(group.split(","))
        label = (arguments.labels[index] if arguments.labels
                 and index < len(arguments.labels) else Path(group).stem)
        print(f"{label}: 뷰 {data['views']}, 열화상 {data['thermal_size']}")
        names.append(label)
        results.append(cmp.evaluate(data))
    print()
    cmp.render(names, results)
    return 0


def capture(arguments) -> int:
    sys.argv = ["capture.py"] + arguments.rest
    runpy.run_path(str(ROOT / "tools" / "paper_calib" / "capture.py"),
                   run_name="__main__")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    c = sub.add_parser("capture", help="Gazebo 에서 자세를 훑으며 뷰 수집")
    c.add_argument("rest", nargs=argparse.REMAINDER)
    c.set_defaults(run=capture)

    s = sub.add_parser("solve", help="수집한 뷰로 외부파라미터 추정")
    s.add_argument("--views", nargs="+", default=[str(RESULTS / "paper_views.npz")],
                   help="여러 개를 주면 하나의 문제로 합쳐 푼다")
    s.add_argument("--init", choices=("stereo", "cad"), default="stereo",
                   help="stereo = 논문과 같은 stereoCalibrate 워밍, cad = 도면값")
    s.set_defaults(run=solve)

    c = sub.add_parser("compare", help="여러 데이터셋을 같은 최적화로 비교")
    c.add_argument("--views", nargs="+", required=True,
                   help="한 항목에 쉼표로 여러 npz 를 주면 합쳐서 하나로 센다")
    c.add_argument("--labels", nargs="*", default=None)
    c.set_defaults(run=compare)

    arguments = parser.parse_args()
    return arguments.run(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
