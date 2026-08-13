"""Build the dual-density calibration board the paper method needs.

The paper's problem is that one pattern cannot serve both cameras when their
resolutions differ by orders of magnitude: a grid fine enough to exploit the
RGB sensor is unresolvable in thermal, and a grid coarse enough for thermal
wastes the RGB sensor. Its answer is two patterns of different density sharing
one physical surface, with a fixed rule saying which RGB point is which thermal
point.

Here both patterns come out of one set of tiles. A tile is one RGB square, and
its colour alternates on the fine grid; its *temperature* alternates on a grid
twice as coarse. The RGB camera therefore sees a 12 x 8 chequerboard and the
thermal camera sees a 6 x 4 one, on the same plane, at the same instant, with
the coarse corners sitting exactly on every other fine corner:

    RGB corner index = 2 * thermal corner index + 1

which is the paper's correspondence rule, here true by construction rather than
by measurement.

Two deliberate departures from the paper, both forced by this robot:

* No ArUco. The paper puts a ChArUco board on the RGB side, which needs its
  markers to survive detection. Its RGB camera is 2028 x 1520; ours is
  640 x 480, and at the board sizes that fit a 57 degree thermal field a
  DICT_5X5 marker lands around 12 px - below reliable decode. Plain
  chequerboard corners have no such floor. What is lost is the marker identity
  that resolves a 180 degree board flip. That turns out not to matter: the two
  cameras are 68 mm apart and see the same view, so they flip together, and a
  flip of both is absorbed exactly by the per-view board pose the optimiser
  already solves for. detect.py checks that they agree.
* No OLED. The paper switches its display between patterns and waits ten
  seconds for the surface to reach thermal equilibrium. Gazebo sets a
  temperature per visual, so both patterns exist at once and no sequencing or
  settling is needed.

    python3 tools/paper_calib/target.py
    python3 tools/paper_calib/target.py --fine-cols 16 --fine-rows 8   # paper's 8x4 thermal
"""
from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent.parent / "src" / "hazard_guard_simulation"
MODEL = PACKAGE / "models" / "paper_calibration_target"

PLUGIN = "ignition-gazebo-thermal-system"
PLUGIN_NAME = "ignition::gazebo::systems::Thermal"

# One RGB square. The coarse thermal square is twice this.
#
# Sized so the whole board fits the thermal field from 0.5 m, because working
# distance is the lever on translation accuracy: a 68 mm baseline shows up as
# parallax of f*t/Z, so halving the range doubles the signal that separates
# translation from rotation. The first version used 50 mm squares on a
# 600 x 200 mm board, which could not be brought closer than about 1.1 m
# without the pattern leaving frame, and its translation error sat at 3.5 mm
# against 1.9 mm for the circle-board run at 0.5-0.75 m.
#
# At 25 mm the thermal square is 50 mm, and the board is 300 x 200 mm. The
# thermal field is 1.086*Z wide and 0.815*Z tall, so at 0.5 m the board covers
# 55% of the width and 49% of the height - enough margin left for the lateral
# and vertical offsets the pose plan needs, and for tilting it. The thermal
# square spans 14.7 px at 0.5 m and 9.2 px at 0.8 m; the detector took 8.2 px
# on the first attempt during the range sweep, so both ends have room.
#
# The grid stays 12 x 8. Changing the board size and the grid at once would
# leave no way to say which of them moved the result.
SQUARE = 0.025
FINE_COLUMNS, FINE_ROWS = 12, 8

# How many fine squares make one thermal square. The paper's rule assumes 2 and
# so does the index mapping; it is named rather than written as a literal so
# the assumption is visible.
COARSE = 2

PLATE_THICKNESS = 0.005
TILE_THICKNESS = 0.001

COLD_C = 15.0   # room temperature
HOT_C = 45.0    # a heated backing panel; the 30 C step survives the render
                # path's ~0.3 C quantisation with room to spare

WHITE = "0.92 0.92 0.92"
BLACK = "0.05 0.05 0.05"


def thermal(parent, celsius: float) -> None:
    plugin = ElementTree.SubElement(parent, "plugin", filename=PLUGIN, name=PLUGIN_NAME)
    ElementTree.SubElement(plugin, "temperature").text = f"{celsius + 273.15:.2f}"


def material(parent, rgb: str) -> None:
    element = ElementTree.SubElement(parent, "material")
    ElementTree.SubElement(element, "ambient").text = f"{rgb} 1"
    ElementTree.SubElement(element, "diffuse").text = f"{rgb} 1"
    # Near-zero specular: a highlight sliding across the board as it turns
    # moves the apparent corner, and the corner is the measurement.
    ElementTree.SubElement(element, "specular").text = "0.02 0.02 0.02 1"


def box(parent, name: str, size: str, pose: str):
    visual = ElementTree.SubElement(parent, "visual", name=name)
    ElementTree.SubElement(visual, "pose").text = pose
    geometry = ElementTree.SubElement(visual, "geometry")
    ElementTree.SubElement(ElementTree.SubElement(geometry, "box"), "size").text = size
    ElementTree.SubElement(visual, "cast_shadows").text = "false"
    return visual


def build(fine_columns: int, fine_rows: int, square: float):
    width = fine_columns * square
    height = fine_rows * square

    sdf = ElementTree.Element("sdf", version="1.7")
    model = ElementTree.SubElement(sdf, "model", name="paper_calibration_target")
    ElementTree.SubElement(model, "static").text = "true"
    link = ElementTree.SubElement(model, "link", name="link")

    # The board faces +x and stands upright, so the model pose is the board
    # pose - the same convention the circle target uses, which is what lets the
    # existing pose-driving code be reused unchanged.
    backing = box(link, "backing", f"{PLATE_THICKNESS} {width:.4f} {height:.4f}", "0 0 0 0 0 0")
    material(backing, BLACK)
    thermal(backing, COLD_C)

    collision = ElementTree.SubElement(link, "collision", name="collision")
    geometry = ElementTree.SubElement(collision, "geometry")
    ElementTree.SubElement(ElementTree.SubElement(geometry, "box"), "size").text = (
        f"{PLATE_THICKNESS} {width:.4f} {height:.4f}"
    )

    front = (PLATE_THICKNESS + TILE_THICKNESS) / 2
    for row in range(fine_rows):
        for column in range(fine_columns):
            # y grows to the left in Gazebo, so column 0 sits at +y and the
            # detector's column order matches image left-to-right when the
            # board faces the camera.
            y = width / 2 - (column + 0.5) * square
            z = height / 2 - (row + 0.5) * square
            tile = box(
                link, f"tile_{row}_{column}",
                f"{TILE_THICKNESS} {square:.4f} {square:.4f}",
                f"{front:.4f} {y:.4f} {z:.4f} 0 0 0",
            )
            material(tile, WHITE if (row + column) % 2 == 0 else BLACK)
            # The thermal pattern is the same parity taken on the coarse grid,
            # so all COARSE x COARSE tiles inside one thermal square share a
            # temperature and the thermal camera cannot see the fine pattern.
            hot = ((row // COARSE) + (column // COARSE)) % 2 == 0
            thermal(tile, HOT_C if hot else COLD_C)

    return sdf, width, height


def specification(fine_columns: int, fine_rows: int, square: float, width: float, height: float):
    """Everything the detector and the optimiser need, in one place.

    Interior corners, not squares: a chequerboard of N squares has N-1 interior
    corners, and every detector in this pipeline counts corners.
    """
    rgb_corners = (fine_columns - 1, fine_rows - 1)
    tir_corners = (fine_columns // COARSE - 1, fine_rows // COARSE - 1)
    # The paper's rule. Written out rather than recomputed downstream so there
    # is one place to look when a correspondence turns out to be wrong.
    matches = [
        {"tir": [m, n], "rgb": [COARSE * m + 1, COARSE * n + 1]}
        for n in range(tir_corners[1])
        for m in range(tir_corners[0])
    ]
    return {
        "fine_columns": fine_columns,
        "fine_rows": fine_rows,
        "square_m": square,
        "coarse_factor": COARSE,
        "tir_square_m": square * COARSE,
        "rgb_corners": list(rgb_corners),
        "tir_corners": list(tir_corners),
        "matched_points": len(matches),
        "correspondence": matches,
        "width_m": round(width, 4),
        "height_m": round(height, 4),
        "cold_c": COLD_C,
        "hot_c": HOT_C,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=str(MODEL))
    parser.add_argument("--fine-cols", type=int, default=FINE_COLUMNS)
    parser.add_argument("--fine-rows", type=int, default=FINE_ROWS)
    parser.add_argument("--square", type=float, default=SQUARE)
    arguments = parser.parse_args()

    if arguments.fine_cols % COARSE or arguments.fine_rows % COARSE:
        print(f"fine-cols 와 fine-rows 는 {COARSE} 의 배수여야 합니다 "
              f"(열화상 격자가 정확히 절반이 되어야 대응 규칙이 성립)")
        return 1

    output = Path(arguments.output)
    output.mkdir(parents=True, exist_ok=True)
    sdf, width, height = build(arguments.fine_cols, arguments.fine_rows, arguments.square)
    spec = specification(arguments.fine_cols, arguments.fine_rows, arguments.square,
                         width, height)

    ElementTree.indent(sdf, space="  ")
    (output / "model.sdf").write_text(
        ElementTree.tostring(sdf, encoding="unicode", xml_declaration=True) + "\n")
    (output / "model.config").write_text(
        "<?xml version='1.0'?>\n"
        "<model>\n"
        "  <name>paper_calibration_target</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version='1.7'>model.sdf</sdf>\n"
        "  <description>\n"
        f"    Dual-density board: {arguments.fine_cols}x{arguments.fine_rows} RGB\n"
        f"    chequerboard, {spec['tir_corners'][0] + 1}x{spec['tir_corners'][1] + 1}\n"
        "    thermal chequerboard on the same tiles.\n"
        "    Generated by tools/paper_calib/target.py - do not hand-edit.\n"
        "  </description>\n"
        "</model>\n")
    (output / "target.json").write_text(json.dumps(spec, indent=2) + "\n")

    print(f"판       {width * 1000:.0f} x {height * 1000:.0f} mm")
    print(f"RGB      {arguments.fine_cols} x {arguments.fine_rows} 칸, "
          f"{arguments.square * 1000:.0f} mm  ->  코너 "
          f"{spec['rgb_corners'][0]} x {spec['rgb_corners'][1]}")
    print(f"열화상   {spec['tir_corners'][0] + 1} x {spec['tir_corners'][1] + 1} 칸, "
          f"{spec['tir_square_m'] * 1000:.0f} mm  ->  코너 "
          f"{spec['tir_corners'][0]} x {spec['tir_corners'][1]}")
    print(f"대응점   {spec['matched_points']} 개/뷰  (rgb = {COARSE}*tir + 1)")
    print(f"온도     {COLD_C} / {HOT_C} C")
    print(f"출력     {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
