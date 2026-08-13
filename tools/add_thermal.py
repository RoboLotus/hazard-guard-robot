"""Give a demo world a thermal signature the camera can actually read.

Gazebo renders every surface at the world's ambient temperature unless a visual
carries the Thermal system plugin. A world without one is a flat field to the
thermal camera - useless for checking that a registration or a detector works,
because a wrong result looks exactly like a right one.

Two treatments:

  equipment    one temperature each, 50-60 C
  background   a grid of thin tiles over the floor and the lower walls,
               each tile its own temperature in 10-20 C

The background is tiled rather than given one value because a uniform floor
gives the camera nothing to lock onto: wherever it points, the frame has to
have structure. A heat-signature texture would be the tidier way to do it, but
the hall mesh carries no UV coordinates, so every texel maps to the same point
and the surface comes out flat - measured, not assumed.

The 10 C spread is not cosmetic. Neighbouring tiles inside a 2 C band came out
of the camera as one flat field: the render path quantises to about 0.3 C, so
values that close collapse into the same level. A 10/30 C checkerboard read
back as 9.55 and 30.11, which is what set the band below.

Temperatures are drawn from a fixed seed, so the world is reproducible.

Run from anywhere; paths resolve relative to this file.

    python3 tools/add_thermal.py                       # default world
    python3 tools/add_thermal.py --world worlds/x.sdf  # another one
"""
import argparse
import random
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent / "src" / "hazard_guard_simulation"

# Fortress plugin names; the rest of the repo converts gz-sim-* the same way.
PLUGIN = "ignition-gazebo-thermal-system"
PLUGIN_NAME = "ignition::gazebo::systems::Thermal"

BACKGROUND_MODEL = "thermal_background"
BACKGROUND_MIN_C, BACKGROUND_MAX_C = 10.0, 20.0
EQUIPMENT_MIN_C, EQUIPMENT_MAX_C = 50.0, 60.0
PLAIN = {"ground_plane": 15.0, "hall_shell": 15.0}  # tiles do not reach every corner

TILE = 0.30          # at 0.5 m the frame spans a few tiles; 0.4 m filled it with one
TILE_LIFT = 0.002    # clear of the floor plane so the two do not z-fight
WALL_INSET = 0.05    # inside the shell, clear of the wall panels
WALL_TOP = 0.80      # the camera sits at 0.13 m and never looks far above this
SEED = 20260812


def kelvin(celsius):
    return celsius + 273.15


def thermal_plugin(temperature_c, indent):
    plugin = ElementTree.Element("plugin", filename=PLUGIN, name=PLUGIN_NAME)
    ElementTree.SubElement(plugin, "temperature").text = f"{kelvin(temperature_c):.2f}"
    plugin.tail = "\n" + indent
    return plugin


def hall_extent(world):
    """Outer half-extent of the hall shell, from its mesh and world scale."""
    shell = next(m for m in world.iter("model") if m.get("name") == "hall_shell")
    mesh = shell.find(".//geometry/mesh")
    scale = float(mesh.find("scale").text.split()[0])
    obj = PACKAGE / "models" / "hall_shell" / "meshes" / "hall_shell.obj"
    xs, ys = [], []
    for line in obj.read_text().splitlines():
        if line.startswith("v "):
            _, x, y, _z = line.split()[:4]
            xs.append(float(x))
            ys.append(float(y))
    return (max(xs) - min(xs)) / 2 * scale, (max(ys) - min(ys)) / 2 * scale


def tile_visual(name, x, y, z, size, temperature_c):
    visual = ElementTree.Element("visual", name=name)
    ElementTree.SubElement(visual, "pose").text = f"{x:.3f} {y:.3f} {z:.3f} 0 0 0"
    geometry = ElementTree.SubElement(visual, "geometry")
    ElementTree.SubElement(ElementTree.SubElement(geometry, "box"), "size").text = (
        f"{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}"
    )
    # Grey and shadow-free: this layer exists for the thermal camera, and should
    # not show up as a pattern on the floor in RGB.
    material = ElementTree.SubElement(visual, "material")
    ElementTree.SubElement(material, "ambient").text = "0.35 0.35 0.35 1"
    ElementTree.SubElement(material, "diffuse").text = "0.38 0.38 0.38 1"
    ElementTree.SubElement(visual, "cast_shadows").text = "false"
    visual.append(thermal_plugin(temperature_c, "        "))
    visual.text = "\n        "
    visual.tail = "\n      "
    return visual


def frange(start, stop, step):
    value = start
    while value < stop - 1e-9:
        yield value
        value += step


def background_model(world, rng):
    """Tiled floor plus a waist-high band on each wall."""
    half_x, half_y = hall_extent(world)
    x0, x1 = -half_x + WALL_INSET, half_x - WALL_INSET
    y0, y1 = -half_y + WALL_INSET, half_y - WALL_INSET

    model = ElementTree.Element("model", name=BACKGROUND_MODEL)
    ElementTree.SubElement(model, "static").text = "true"
    link = ElementTree.SubElement(model, "link", name="link")
    count = 0

    for x in frange(x0, x1, TILE):
        for y in frange(y0, y1, TILE):
            sx, sy = min(TILE, x1 - x), min(TILE, y1 - y)
            link.append(tile_visual(
                f"floor_{count}", x + sx / 2, y + sy / 2, TILE_LIFT,
                (sx, sy, 0.001), rng.uniform(BACKGROUND_MIN_C, BACKGROUND_MAX_C),
            ))
            count += 1

    for z in frange(TILE_LIFT, WALL_TOP, TILE):
        height = min(TILE, WALL_TOP - z)
        for x in frange(x0, x1, TILE):
            width = min(TILE, x1 - x)
            for y_edge in (y0, y1):
                link.append(tile_visual(
                    f"wall_{count}", x + width / 2, y_edge, z + height / 2,
                    (width, 0.001, height),
                    rng.uniform(BACKGROUND_MIN_C, BACKGROUND_MAX_C),
                ))
                count += 1
        for y in frange(y0, y1, TILE):
            depth = min(TILE, y1 - y)
            for x_edge in (x0, x1):
                link.append(tile_visual(
                    f"wall_{count}", x_edge, y + depth / 2, z + height / 2,
                    (0.001, depth, height),
                    rng.uniform(BACKGROUND_MIN_C, BACKGROUND_MAX_C),
                ))
                count += 1

    model.text = model.tail = "\n    "
    link.text = "\n      "
    link.tail = "\n    "
    return model, count, (half_x * 2, half_y * 2)


def stamp(world_path):
    tree = ElementTree.parse(world_path)
    world = tree.getroot().find("world")
    rng = random.Random(SEED)

    for old in [m for m in world.findall("model") if m.get("name") == BACKGROUND_MODEL]:
        world.remove(old)  # re-running must not stack layers

    report = []
    for model in world.iter("model"):
        name = model.get("name")
        if name == BACKGROUND_MODEL:
            continue
        temperature = PLAIN.get(name)
        if temperature is None:
            temperature = round(rng.uniform(EQUIPMENT_MIN_C, EQUIPMENT_MAX_C), 1)
        for visual in model.iter("visual"):
            for old in visual.findall("plugin"):
                visual.remove(old)
            visual.append(thermal_plugin(temperature, "        "))
        report.append((model.get("name"), f"{temperature} C"))

    background, count, size = background_model(world, rng)
    world.append(background)
    tree.write(world_path, encoding="utf-8", xml_declaration=True)
    return report, count, size


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--world",
        default=str(PACKAGE / "worlds" / "demo_facility_scaled.sdf"),
        help="world SDF to stamp",
    )
    world = Path(parser.parse_args().world)
    report, count, size = stamp(world)
    print(f"월드 {world.name}\n")
    for name, treatment in report:
        print(f"  {name:22s} {treatment}")
    print(
        f"\n  {BACKGROUND_MODEL:22s} 타일 {count}개, "
        f"{BACKGROUND_MIN_C}~{BACKGROUND_MAX_C} C 무작위 "
        f"(홀 {size[0]:.2f} x {size[1]:.2f} m, 타일 {TILE} m)"
    )


if __name__ == "__main__":
    main()
