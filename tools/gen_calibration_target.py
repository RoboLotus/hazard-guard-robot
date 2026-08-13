"""Generate the thermal/RGB calibration target used to check the extrinsic.

The physical target is a matte board with a grid of holes, held in front of
something warm: the holes show the warm background, the board shows room
temperature. Here it is modelled the other way round - a cool plate carrying
warm discs - because the two are identical to the camera (a grid of warm
circles on a cool plane) and Gazebo cannot subtract one shape from another.

What matters for calibration is that the circle centres sit on a plane at a
known spacing, and that is exactly what this produces.

The discs are dark in RGB and warm in thermal, so one target drives both
detectors: cv2.findCirclesGrid sees dark blobs directly in RGB, and in thermal
after inverting the image.

    python3 tools/gen_calibration_target.py
"""
import argparse
import json
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent / "src" / "hazard_guard_simulation"
MODEL = PACKAGE / "models" / "calibration_target"

PLUGIN = "ignition-gazebo-thermal-system"
PLUGIN_NAME = "ignition::gazebo::systems::Thermal"

# Small board, used close. Translation is only observable through parallax, and
# parallax is inversely proportional to range: halving the working distance
# doubles the signal that fixes the up-down and fore-aft offsets, which are the
# two the first version got wrong by 15 mm.
#
# 40 mm circles at 0.5 m still span 11.8 px - the same as 70 mm circles did at
# 0.9 m - and smaller circles also halve the ellipse-centroid bias that made
# tilted views hurt rather than help.
#
# A 320 mm board fits the thermal frame (44.3 deg vertical, camera 0.16 m off
# the floor) from about 0.45 m out.
COLUMNS, ROWS = 4, 4
DIAMETER = 0.040
SPACING = 0.080
MARGIN = 0.020
PLATE_THICKNESS = 0.005
DISC_THICKNESS = 0.002

PLATE_C = 15.0   # room temperature, like the real board
DISC_C = 35.0    # a heating pad behind the real board reads about this


def kelvin(celsius):
    return celsius + 273.15


def thermal(parent, celsius):
    plugin = ElementTree.SubElement(
        parent, "plugin", filename=PLUGIN, name=PLUGIN_NAME
    )
    ElementTree.SubElement(plugin, "temperature").text = f"{kelvin(celsius):.2f}"


def material(parent, rgb):
    element = ElementTree.SubElement(parent, "material")
    ElementTree.SubElement(element, "ambient").text = f"{rgb} 1"
    ElementTree.SubElement(element, "diffuse").text = f"{rgb} 1"
    ElementTree.SubElement(element, "specular").text = "0.05 0.05 0.05 1"


def build():
    width = (COLUMNS - 1) * SPACING + DIAMETER + 2 * MARGIN
    height = (ROWS - 1) * SPACING + DIAMETER + 2 * MARGIN

    sdf = ElementTree.Element("sdf", version="1.7")
    model = ElementTree.SubElement(sdf, "model", name="calibration_target")
    ElementTree.SubElement(model, "static").text = "true"
    link = ElementTree.SubElement(model, "link", name="link")

    # The plate faces +x, standing upright, so the model pose is the board pose.
    plate = ElementTree.SubElement(link, "visual", name="plate")
    ElementTree.SubElement(plate, "pose").text = "0 0 0 0 0 0"
    geometry = ElementTree.SubElement(plate, "geometry")
    ElementTree.SubElement(ElementTree.SubElement(geometry, "box"), "size").text = (
        f"{PLATE_THICKNESS} {width:.4f} {height:.4f}"
    )
    material(plate, "0.92 0.92 0.92")
    ElementTree.SubElement(plate, "cast_shadows").text = "false"
    thermal(plate, PLATE_C)

    collision = ElementTree.SubElement(link, "collision", name="collision")
    geometry = ElementTree.SubElement(collision, "geometry")
    ElementTree.SubElement(ElementTree.SubElement(geometry, "box"), "size").text = (
        f"{PLATE_THICKNESS} {width:.4f} {height:.4f}"
    )

    centres = []
    for row in range(ROWS):
        for column in range(COLUMNS):
            # y grows to the left in Gazebo, so the first column is placed at
            # +y; the detector's column order then matches the image's left to
            # right when the board faces the camera.
            y = (COLUMNS - 1) / 2 * SPACING - column * SPACING
            z = (ROWS - 1) / 2 * SPACING - row * SPACING
            centres.append((y, z))
            disc = ElementTree.SubElement(
                link, "visual", name=f"disc_{row}_{column}"
            )
            ElementTree.SubElement(disc, "pose").text = (
                f"{(PLATE_THICKNESS + DISC_THICKNESS) / 2:.4f} {y:.4f} {z:.4f} "
                "0 1.5707963 0"
            )
            geometry = ElementTree.SubElement(disc, "geometry")
            cylinder = ElementTree.SubElement(geometry, "cylinder")
            ElementTree.SubElement(cylinder, "radius").text = f"{DIAMETER / 2:.4f}"
            ElementTree.SubElement(cylinder, "length").text = f"{DISC_THICKNESS:.4f}"
            material(disc, "0.05 0.05 0.05")
            ElementTree.SubElement(disc, "cast_shadows").text = "false"
            thermal(disc, DISC_C)

    return sdf, width, height, centres


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(MODEL))
    output = Path(parser.parse_args().output)
    output.mkdir(parents=True, exist_ok=True)

    sdf, width, height, centres = build()
    ElementTree.indent(sdf, space="  ")
    (output / "model.sdf").write_text(
        ElementTree.tostring(sdf, encoding="unicode", xml_declaration=True) + "\n"
    )
    (output / "model.config").write_text(
        "<?xml version='1.0'?>\n"
        "<model>\n"
        "  <name>calibration_target</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version='1.7'>model.sdf</sdf>\n"
        "  <description>\n"
        f"    {COLUMNS}x{ROWS} circle grid, {DIAMETER * 1000:.0f} mm circles on a\n"
        f"    {SPACING * 1000:.0f} mm pitch. Generated by tools/gen_calibration_target.py\n"
        "    - do not hand-edit.\n"
        "  </description>\n"
        "</model>\n"
    )
    # The measuring tool reads this rather than repeating the numbers, so the
    # two cannot drift apart.
    (output / "target.json").write_text(json.dumps({
        "columns": COLUMNS, "rows": ROWS,
        "spacing_m": SPACING, "diameter_m": DIAMETER,
        "width_m": round(width, 4), "height_m": round(height, 4),
        "plate_c": PLATE_C, "disc_c": DISC_C,
    }, indent=2) + "\n")

    print(f"판    {width * 1000:.0f} x {height * 1000:.0f} mm")
    print(f"원    지름 {DIAMETER * 1000:.0f} mm, 간격 {SPACING * 1000:.0f} mm, "
          f"{COLUMNS} x {ROWS} = {len(centres)}개")
    print(f"온도  판 {PLATE_C} C / 원 {DISC_C} C")
    print(f"출력  {output}")


if __name__ == "__main__":
    main()
