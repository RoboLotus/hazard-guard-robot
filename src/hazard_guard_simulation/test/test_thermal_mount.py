"""The thermal camera mount has to clear the LiDAR and the RGB-D housing.

The mount is exposed as xacro arguments so calibration experiments can perturb
it. That makes it easy to move the board somewhere it does not belong: above
the RGB-D unit it cuts the forward LiDAR sector, and too close to the housing
it intersects it. Both look fine in RViz and only show up as a mangled scan or
a physics warning much later, so the defaults are checked here.
"""
import re
import xml.etree.ElementTree as ElementTree
from pathlib import Path

ROBOT = (
    Path(__file__).resolve().parent.parent
    / "urdf"
    / "hazard_guard_m1.urdf.xacro"
)
LIDAR_MARGIN = 0.005  # keep the board a clear 5 mm under the scan plane

SOURCE = ROBOT.read_text()
ROOT = ElementTree.parse(ROBOT).getroot()


def arg(name):
    match = re.search(
        rf'<xacro:arg name="{name}" default="([-0-9.]+)"', SOURCE
    )
    assert match, f"missing xacro arg {name}"
    return float(match.group(1))


def joint_origin(name):
    """A joint's xyz, with $(arg ...) resolved to the argument's default.

    The thermal joints are written as arguments so a calibration can be
    applied on top; what the checks below want is the drawing, which is what
    those defaults hold.
    """
    for joint in ROOT.iter("joint"):
        if joint.get("name") == name:
            xyz = joint.find("origin").get("xyz", "0 0 0")
            resolved = re.sub(r"\$\(arg\s+([A-Za-z0-9_]+)\)",
                              lambda m: str(arg(m.group(1))), xyz)
            return [float(v) for v in resolved.split()]
    raise AssertionError(f"missing joint {name}")


def box(link_name, tag):
    """(centre, size) of the first box of `tag` type on a link."""
    for link in ROOT.iter("link"):
        if link.get("name") != link_name:
            continue
        for element in link.iter(tag):
            geometry = element.find("geometry/box")
            if geometry is None:
                continue
            origin = element.find("origin")
            xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            return (
                [float(v) for v in xyz.split()],
                [float(v) for v in geometry.get("size").split()],
            )
    raise AssertionError(f"missing {tag} box on {link_name}")


THERMAL = [arg("thermal_mount_x"), arg("thermal_mount_y"), arg("thermal_mount_z")]
_, THERMAL_SIZE = box("thermal_camera_link", "visual")


def test_thermal_board_stays_below_the_lidar_plane():
    top = THERMAL[2] + THERMAL_SIZE[2] / 2
    plane = joint_origin("lidar_joint")[2]
    assert top + LIDAR_MARGIN <= plane, (
        f"thermal board reaches z {top:.4f}, LiDAR plane is {plane:.4f}"
    )


def test_thermal_board_clears_the_rgbd_housing():
    camera = joint_origin("depth_camera_joint")
    centre, size = box("depth_camera_link", "collision")
    housing_edge = abs(camera[1] + centre[1]) + size[1] / 2
    gap = abs(THERMAL[1]) - THERMAL_SIZE[1] / 2 - housing_edge
    assert gap >= 0, f"thermal board overlaps the camera housing by {-gap:.4f} m"


def test_baseline_is_a_pure_sideways_offset():
    """Optical frames level in x and z; only y separates them."""
    camera = joint_origin("depth_camera_joint")
    depth_optical = joint_origin("depth_camera_optical_joint")
    thermal_optical = joint_origin("thermal_camera_optical_joint")
    depth = [camera[i] + depth_optical[i] for i in range(3)]
    thermal = [THERMAL[i] + thermal_optical[i] for i in range(3)]
    assert abs(depth[0] - thermal[0]) < 1e-6, "optical frames differ in x"
    assert abs(depth[2] - thermal[2]) < 1e-6, "optical frames differ in z"
    assert abs(thermal[1] - depth[1]) > 0.03, "baseline too short to calibrate"


if __name__ == "__main__":
    test_thermal_board_stays_below_the_lidar_plane()
    test_thermal_board_clears_the_rgbd_housing()
    test_baseline_is_a_pure_sideways_offset()
    camera = joint_origin("depth_camera_joint")
    depth_optical = joint_origin("depth_camera_optical_joint")
    thermal_optical = joint_origin("thermal_camera_optical_joint")
    print(
        "ok  baseline y="
        f"{THERMAL[1] + thermal_optical[1] - camera[1] - depth_optical[1]:.4f} m"
        f", board top z={THERMAL[2] + THERMAL_SIZE[2] / 2:.4f}"
        f", lidar z={joint_origin('lidar_joint')[2]:.4f}"
    )
