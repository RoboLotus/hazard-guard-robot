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

from paper_calib import optimize as opt  # noqa: E402


def load(path: Path) -> dict:
    raw = np.load(path, allow_pickle=False)
    return {
        "views": len(raw["rgb_matched"]),
        "labels": [str(v) for v in raw["labels"]],
        "tir_how": [str(v) for v in raw["tir_how"]],
        "rgb_corners": raw["rgb_corners"].astype(np.float64),
        "rgb_matched": raw["rgb_matched"].astype(np.float64),
        "tir_matched": raw["tir_matched"].astype(np.float64),
        "object_points": raw["object_points"].astype(np.float64),
        "object_matched": raw["object_matched"].astype(np.float64),
        "rgb_k": raw["rgb_k"].reshape(3, 3),
        "thermal_k": raw["thermal_k"].reshape(3, 3),
        "rgb_size": tuple(int(v) for v in raw["rgb_size"]),
        "thermal_size": tuple(int(v) for v in raw["thermal_size"]),
    }


def solve(arguments) -> int:
    data = load(Path(arguments.views))
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
    s.add_argument("--views", default=str(RESULTS / "paper_views.npz"))
    s.add_argument("--init", choices=("stereo", "cad"), default="stereo",
                   help="stereo = 논문과 같은 stereoCalibrate 워밍, cad = 도면값")
    s.set_defaults(run=solve)

    arguments = parser.parse_args()
    return arguments.run(arguments)


if __name__ == "__main__":
    raise SystemExit(main())
