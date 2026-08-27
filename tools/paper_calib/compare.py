"""Run the same Mode A fit on several datasets and put the numbers side by side.

A calibration change is an experiment, and an experiment needs a control. The
board was shrunk and the working range halved to improve translation
observability; whether that worked is not answerable from one run's numbers,
only from the same optimiser on both datasets with the same ground truth.

Reprojection error alone cannot answer it either. A fit can drive residuals
down while sitting in the wrong place if a parameter is weakly observed - the
residual barely notices where that parameter goes. So the table carries three
things that have to move together to call it an improvement: how close the
answer is to the truth, how large the residuals are, and how sharply the
residual responds to each parameter. The third is the one that says whether the
first is luck.
"""
from __future__ import annotations

import numpy as np

from . import optimize as opt

PARAMETERS = ("tx", "ty", "tz", "roll", "pitch", "yaw")


def evaluate(data: dict) -> dict:
    params, report = opt.solve(data, init="stereo", verbose=False)
    base, rows = opt.sensitivity(params, data)
    translation = np.array(report["translation_mm"])
    rpy = np.array(report["rpy_deg"])
    return {
        "views": data["views"],
        "attempted": data.get("attempted"),
        "translation_error_mm": translation - opt.GROUND_TRUTH_MM,
        "translation_error_norm_mm": float(
            np.linalg.norm(translation - opt.GROUND_TRUTH_MM)),
        "rotation_error_deg": rpy - opt.GROUND_TRUTH_RPY_DEG,
        "translation_mm": translation,
        "rpy_deg": rpy,
        "rgb_rms_px": report["rgb_rms_px"],
        "tir_rms_px": report["tir_rms_px"],
        "total_rms_px": report["total_rms_px"],
        "sensitivity": {row["parameter"]: row["delta_tir_rms_px"] for row in rows},
        "sensitivity_base_px": base,
        "per_frame_tir_px": report["per_frame_tir_px"],
        "labels": data["labels"],
    }


def _row(name: str, values, formatter="{:>12}"):
    return f"  {name:26s}" + "".join(formatter.format(v) for v in values)


def render(names, results) -> None:
    print("=" * (28 + 12 * len(names)))
    print(_row("", names))
    print("-" * (28 + 12 * len(names)))

    print(_row("채택 뷰", [
        f"{r['views']}/{r['attempted']}" if r["attempted"] else str(r["views"])
        for r in results]))
    print()
    for axis, name in enumerate(("tx 오차 mm", "ty 오차 mm", "tz 오차 mm")):
        print(_row(name, [f"{r['translation_error_mm'][axis]:+.2f}" for r in results]))
    print(_row("이동 오차 크기 mm",
               [f"{r['translation_error_norm_mm']:.2f}" for r in results]))
    print()
    for axis, name in enumerate(("roll 오차 deg", "pitch 오차 deg", "yaw 오차 deg")):
        print(_row(name, [f"{r['rotation_error_deg'][axis]:+.3f}" for r in results]))
    print()
    print(_row("RGB RMS px", [f"{r['rgb_rms_px']:.4f}" for r in results]))
    print(_row("열화상 RMS px", [f"{r['tir_rms_px']:.4f}" for r in results]))
    print(_row("전체 RMS px", [f"{r['total_rms_px']:.4f}" for r in results]))
    print()
    print(_row("민감도 (열화상 RMS 변화)", ["" for _ in results]))
    for parameter in PARAMETERS:
        step = "+1mm" if parameter in ("tx", "ty", "tz") else "+0.1deg"
        print(_row(f"  {parameter} {step}",
                   [f"{r['sensitivity'][parameter]:+.4f}" for r in results]))
    print()
    # Ratios, not absolutes: an absolute sensitivity moves with the residual
    # scale and the point count, while how far tz sits below tx does not, and
    # that gap is what leaves tz free.
    #
    # The square root is the number to read. Residual RMS at the optimum is not
    # zero, so a small perturbation gives RMS ~ sqrt(RMS0^2 + |J d|^2), which
    # grows as |J d|^2 while the perturbation stays small against RMS0 -
    # measured here as a 13.6x rise for a 4x step on tz against 7.7x on tx, the
    # weak axis being the more quadratic one. The printed delta is therefore
    # closer to the square of the observability than to the observability, and
    # taking the root undoes that: a raw tz/tx of 0.067 is a real gap of about
    # four, not fifteen.
    for weak, strong in (("tz", "tx"), ("yaw", "roll")):
        print(_row(f"{weak} / {strong} 민감도 비", [
            f"{r['sensitivity'][weak] / r['sensitivity'][strong]:.3f}"
            if r["sensitivity"][strong] else "-" for r in results]))
        print(_row(f"  제곱근 (관측성 비)", [
            f"{np.sqrt(max(r['sensitivity'][weak] / r['sensitivity'][strong], 0)):.3f}"
            if r["sensitivity"][strong] else "-" for r in results]))
    print("=" * (28 + 12 * len(names)))
