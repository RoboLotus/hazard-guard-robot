#!/usr/bin/env python3
"""Turn a solved thermal-depth extrinsic into the mount pose the URDF wants.

The calibration answers a question about optical frames: where a point in the
depth optical frame lands in the thermal one. The URDF asks a different
question - where the thermal camera link sits on base_link - and TF is built
from the second. This walks the answer back through the two optical joints so
the TF chain reproduces the measured extrinsic exactly.

That matters here for a reason beyond tidiness. None of the <sensor> blocks in
the URDF carry a <pose>, so Gazebo renders each camera at its link origin and
the optical joints never reach the renderer. TF says the baseline is a clean
sideways 68 mm; the pictures say (68, 12, -21). Writing the measured extrinsic
back is what closes that gap, and it is the same operation a real robot needs
once its own calibration lands - there the gap comes from the bracket rather
than from a missing <pose>, and the arithmetic does not care which.

The joint constants are read out of the xacro rather than copied, so moving a
camera in the URDF cannot silently leave this file behind.

No numpy, no ROS - it runs on the host, like target.py does.

    python3 tools/apply_calibration.py                    # newest paper result
    python3 tools/apply_calibration.py --report runtime/calibration/x.json
    python3 tools/apply_calibration.py --mode C1
    python3 tools/apply_calibration.py selftest
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ElementTree
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / "src/hazard_guard_simulation/urdf/hazard_guard_m1.urdf.xacro"
RESULTS = ROOT / "runtime" / "calibration"
OUTPUT = ROOT / "src/hazard_guard_simulation/config/thermal_extrinsic.yaml"

ARGUMENT_NAMES = ["thermal_optical_x", "thermal_optical_y", "thermal_optical_z",
                  "thermal_optical_roll", "thermal_optical_pitch",
                  "thermal_optical_yaw"]


# --- 4x4 rigid transforms as (rotation, translation) pairs -------------------
# Nine multiplies is not worth a dependency, and staying stdlib is what lets
# this run outside the container.

def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> list:
    """Fixed-axis roll-pitch-yaw, the convention URDF <origin rpy> uses."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rpy_from_rotation(rotation: list) -> tuple:
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    roll = math.atan2(rotation[2][1], rotation[2][2])
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    return roll, pitch, yaw


def multiply(a: list, b: list) -> list:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def apply_rotation(rotation: list, vector) -> list:
    return [sum(rotation[i][k] * vector[k] for k in range(3)) for i in range(3)]


def transpose(rotation: list) -> list:
    return [[rotation[j][i] for j in range(3)] for i in range(3)]


def compose(first: tuple, second: tuple) -> tuple:
    """first * second, both as (rotation, translation)."""
    rotation_a, translation_a = first
    rotation_b, translation_b = second
    return (multiply(rotation_a, rotation_b),
            [translation_a[i] + apply_rotation(rotation_a, translation_b)[i]
             for i in range(3)])


def invert(transform: tuple) -> tuple:
    rotation, translation = transform
    inverse = transpose(rotation)
    moved = apply_rotation(inverse, translation)
    return inverse, [-value for value in moved]


IDENTITY = ([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], [0.0, 0.0, 0.0])


# --- what the URDF says ------------------------------------------------------

ARGUMENT_PATTERN = re.compile(r"\$\(arg\s+([A-Za-z0-9_]+)\)")

# base_link to each optical frame. Everything the conversion needs, and
# nothing that would drag in xacro property evaluation.
CHAIN = ("depth_camera_joint", "depth_camera_optical_joint",
         "thermal_camera_joint", "thermal_camera_optical_joint")


def xacro_arguments(root) -> dict:
    """Every <xacro:arg> default, by name."""
    return {element.get("name"): element.get("default")
            for element in root.iter()
            if element.tag.endswith("}arg") and element.get("name")}


def numbers(text: str, arguments: dict) -> list:
    """Three floats from an origin attribute, resolving $(arg ...) defaults."""
    resolved = ARGUMENT_PATTERN.sub(
        lambda match: arguments.get(match.group(1), match.group(0)), text)
    return [float(value) for value in resolved.split()]


def joint_origins() -> dict:
    """Every fixed camera joint's origin, straight out of the xacro.

    Origins written as $(arg ...) resolve to the argument's default, which is
    the mounting drawing - the value in force when no calibration is applied.
    """
    root = ElementTree.parse(URDF).getroot()
    arguments = xacro_arguments(root)
    origins = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        # Only the camera chain. Other joints carry ${...} property expressions
        # that would need a xacro run to evaluate, and none of them are on the
        # path between the two optical frames.
        if name not in CHAIN or origin is None:
            continue
        translation = numbers(origin.get("xyz", "0 0 0"), arguments)
        angles = numbers(origin.get("rpy", "0 0 0"), arguments)
        origins[name] = (rotation_from_rpy(*angles), translation)
    missing = [name for name in CHAIN if name not in origins]
    if missing:
        raise SystemExit(f"{URDF.name} 에 조인트가 없습니다: {', '.join(missing)}")
    return origins


def urdf_defaults() -> list:
    """The optical arguments the xacro falls back to when nothing is applied."""
    arguments = xacro_arguments(ElementTree.parse(URDF).getroot())
    return [float(arguments[name]) for name in ARGUMENT_NAMES]


# --- the conversion ----------------------------------------------------------

def base_to_depth_optical(origins: dict) -> tuple:
    return compose(origins["depth_camera_joint"],
                   origins["depth_camera_optical_joint"])


def solve_optical(translation_m, rpy_rad, origins: dict) -> list:
    """Optical-joint arguments that make TF reproduce the measured extrinsic.

    The measurement is thermal_optical <- depth_optical. Chase it backwards:
    base <- thermal_optical is base <- depth_optical undone by the
    measurement, and the optical joint is what is left once the bracket the
    housing hangs from is taken off the front.
    """
    measured = (rotation_from_rpy(*rpy_rad), list(translation_m))
    base_to_thermal_optical = compose(base_to_depth_optical(origins),
                                      invert(measured))
    optical = compose(invert(origins["thermal_camera_joint"]),
                      base_to_thermal_optical)
    rotation, position = optical
    return list(position) + list(rpy_from_rotation(rotation))


def forward(optical: list, origins: dict) -> tuple:
    """The extrinsic a given optical joint produces - solve_optical backwards."""
    optical_joint = (rotation_from_rpy(*optical[3:6]), list(optical[0:3]))
    base_to_thermal_optical = compose(origins["thermal_camera_joint"],
                                      optical_joint)
    return compose(invert(base_to_thermal_optical),
                   base_to_depth_optical(origins))


# --- reports -----------------------------------------------------------------

def newest_report() -> Path:
    reports = sorted(RESULTS.glob("paper_mode*.json"))
    if not reports:
        raise SystemExit(
            f"{RESULTS} 에 paper_mode*.json 이 없습니다. 먼저 캘리브레이션을 돌리세요:\n"
            "  python3 tools/calibrate_paper.py solve --views ...")
    return reports[-1]


def read_extrinsic(path: Path, mode: str) -> tuple:
    """(translation in metres, rpy in radians, label) from a result JSON."""
    report = json.loads(path.read_text())
    label = path.name
    if "modes" in report:
        # The modes file keys read "A 무구속", "C1 회전=0 (유효)" and so on; the
        # mode is the first word.
        matches = [key for key in report["modes"] if key.split()[0] == mode]
        if not matches:
            raise SystemExit(
                f"{path.name} 에 모드 {mode} 가 없습니다. "
                f"있는 것: {', '.join(sorted(report['modes']))}")
        label = f"{path.name} [{matches[0]}]"
        report = report["modes"][matches[0]]
    if "translation_mm" not in report:
        raise SystemExit(f"{path.name} 에 translation_mm 이 없습니다.")
    # The board tool calls it rotation_deg, the paper tool rpy_deg; both are
    # fixed-axis degrees.
    angles = report.get("rpy_deg", report.get("rotation_deg"))
    if angles is None:
        raise SystemExit(f"{path.name} 에 회전이 없습니다.")
    return ([value / 1000.0 for value in report["translation_mm"]],
            [math.radians(value) for value in angles],
            label)


def write_yaml(optical: list, source: str, path: Path) -> None:
    lines = [
        "# Generated by tools/apply_calibration.py - do not edit by hand.",
        f"# Source: {source}",
        "#",
        "# Pose of thermal_camera_optical_frame in thermal_camera_link, chosen",
        "# so the TF chain reproduces the measured depth-to-thermal extrinsic.",
        "# Delete this file to fall back to the drawing values in the xacro.",
    ]
    lines += [f"{name}: {value:.9g}" for name, value in zip(ARGUMENT_NAMES, optical)]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def report_change(optical: list, defaults: list, origins: dict,
                  translation_m, rpy_rad) -> None:
    print(f"{'인자':<24}{'현재(도면)':>13}{'적용값':>13}{'차이':>13}")
    for index, name in enumerate(ARGUMENT_NAMES):
        scale, unit = (1000.0, "mm") if index < 3 else (180.0 / math.pi, "deg")
        print(f"{name:<24}{defaults[index] * scale:>10.3f} {unit:<3}"
              f"{optical[index] * scale:>10.3f} {unit:<3}"
              f"{(optical[index] - defaults[index]) * scale:>+10.3f} {unit:<3}")

    def line(label, transform):
        rotation, translation = transform
        angles = rpy_from_rotation(rotation)
        print(f"  {label:<10}"
              f"T = [{translation[0] * 1000:7.2f} {translation[1] * 1000:7.2f} "
              f"{translation[2] * 1000:7.2f}] mm   "
              f"RPY = [{math.degrees(angles[0]):6.3f} "
              f"{math.degrees(angles[1]):6.3f} {math.degrees(angles[2]):6.3f}] deg")

    print("\nTF 가 발행하는 열화상 <- 뎁스 외부파라미터")
    line("적용 전", forward(defaults, origins))
    line("적용 후", (rotation_from_rpy(*rpy_rad), list(translation_m)))
    before = forward(defaults, origins)[1]
    print(f"  {'이동량':<10}{math.dist(before, translation_m) * 1000:.2f} mm")


# --- self-check --------------------------------------------------------------

def selftest() -> None:
    origins = joint_origins()
    defaults = urdf_defaults()

    # The mounting drawing must come back out as the xacro defaults. This is
    # the whole chain checked against numbers nobody computed with this code:
    # a pure sideways 68 mm is what the URDF says it was built to give, so
    # feeding that back in has to reproduce the optical joint verbatim.
    drawing = solve_optical([0.068, 0.0, 0.0], [0.0, 0.0, 0.0], origins)
    for index, name in enumerate(ARGUMENT_NAMES):
        assert abs(drawing[index] - defaults[index]) < 1e-9, (
            f"{name}: 도면 (68, 0, 0) 이 xacro 기본값을 복원하지 못함 "
            f"{drawing[index]} != {defaults[index]}")

    # Any extrinsic must survive the round trip, rotations included - a sign
    # slip in an inverse would pass the check above, where R comes out I.
    measured = ([0.06846, 0.01190, -0.02153],
                [math.radians(-0.0372), math.radians(-0.0302), math.radians(0.0241)])
    optical = solve_optical(*measured, origins)
    rotation, translation = forward(optical, origins)
    for index in range(3):
        assert abs(translation[index] - measured[0][index]) < 1e-12, "왕복 이동 불일치"
    for value, expected in zip(rpy_from_rotation(rotation), measured[1]):
        assert abs(value - expected) < 1e-12, "왕복 회전 불일치"

    # The correction has to stay inside the housing. If it ever needs more than
    # a couple of centimetres the bracket moved, not the lens, and writing it
    # to the optical joint would be hiding a mechanical change in an optical
    # number - the calibration is right and the URDF is wrong.
    shift = math.dist(optical[0:3], defaults[0:3])
    assert shift < 0.05, f"광학 보정 {shift * 1000:.1f} mm - 하우징 밖, 마운트를 확인"

    # The gap this closes: parallax TF was not reporting, because the optical
    # joints never reach the renderer. At 0.5 m it is several thermal pixels.
    before = forward(defaults, origins)[1]
    moved = math.dist(before, measured[0])
    assert moved > 0.02, f"보정이 아무것도 바꾸지 않음 ({moved * 1000:.2f} mm)"

    # And the bracket must not move: the sensor renders from the link origin,
    # so a mount that shifted would drag the rendered camera along and the
    # correction would chase its own tail.
    assert "thermal_mount" not in " ".join(ARGUMENT_NAMES), "마운트를 건드리면 안 됨"

    print(f"selftest 통과 - 도면 복원, 왕복 오차 < 1e-12, "
          f"광학 보정 {shift * 1000:.2f} mm, TF 이동량 {moved * 1000:.2f} mm")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="apply",
                        choices=["apply", "selftest"])
    parser.add_argument("--report", type=Path,
                        help="캘리브레이션 결과 JSON (기본: 가장 최근 paper_mode*)")
    parser.add_argument("--mode", default="A",
                        help="modes 파일일 때 고를 모드 (기본 A)")
    parser.add_argument("--out", type=Path, default=OUTPUT)
    parser.add_argument("--dry-run", action="store_true",
                        help="계산만 하고 파일은 쓰지 않음")
    arguments = parser.parse_args()

    if arguments.command == "selftest":
        selftest()
        return

    path = arguments.report or newest_report()
    translation_m, rpy_rad, label = read_extrinsic(path, arguments.mode)
    origins = joint_origins()
    optical = solve_optical(translation_m, rpy_rad, origins)

    report_change(optical, urdf_defaults(), origins, translation_m, rpy_rad)
    if arguments.dry_run:
        print("\n--dry-run: 파일 안 씀")
        return
    write_yaml(optical, label, arguments.out)
    print(f"\n{arguments.out.relative_to(ROOT)} 기록됨 (출처: {label})")
    print("colcon build --packages-select hazard_guard_simulation 후 런치하면 반영됩니다.")


if __name__ == "__main__":
    sys.exit(main())
