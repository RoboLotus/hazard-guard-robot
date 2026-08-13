"""Every camera must own its CameraInfo topic and be bridged.

Without an explicit <camera_info_topic>, Fortress derives one by dropping the
last element of the image topic and appending /camera_info - so /camera,
/depth_camera and /thermal_camera all land on a single /camera_info and
overwrite each other's intrinsics. That failure is silent: the bridge runs and
the topic ticks, it just carries whichever camera published last. Calibration
needs them apart, so the wiring is checked here rather than in the simulator.
"""
import re
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
ROBOT = PACKAGE / "urdf" / "hazard_guard_m1.urdf.xacro"
LAUNCH = PACKAGE / "launch" / "simulation.launch.py"
CAMERA_TYPES = {"camera", "depth_camera", "thermal_camera"}


def cameras() -> dict:
    """sensor name -> (image topic, camera_info topic) for every camera."""
    root = ElementTree.parse(ROBOT).getroot()
    found = {}
    for sensor in root.iter("sensor"):
        if sensor.get("type") not in CAMERA_TYPES:
            continue
        name = sensor.get("name")
        topic = sensor.find("topic")
        assert topic is not None, f"{name} publishes no topic"
        info = sensor.find("camera/camera_info_topic")
        assert info is not None, (
            f"{name} has no <camera_info_topic>; its intrinsics would be "
            "derived onto a topic shared with the other cameras"
        )
        found[name] = (topic.text.strip(), info.text.strip())
    return found


def bridge_arguments() -> str:
    """The bridge list with its line wrapping joined back up."""
    return re.sub(r'"\s*\n\s*"', "", LAUNCH.read_text())


def test_cameras_exist():
    assert len(cameras()) >= 2, "calibration needs two camera streams"


def test_info_topics_are_unique():
    infos = [info for _, info in cameras().values()]
    assert len(set(infos)) == len(infos), f"CameraInfo topics collide: {infos}"


def test_declared_info_topic_matches_the_derived_one():
    """The declaration is not enough on its own.

    Only the plain CameraSensor reads <camera_info_topic> in Fortress. The
    depth and thermal sensors ignore it and derive the topic from the image
    topic, so a declared /depth_camera/camera_info sitting on a root-level
    /depth_camera image topic silently publishes nothing while a second camera
    takes over the shared /camera_info.
    """
    for name, (image, declared) in cameras().items():
        derived = image.rsplit("/", 1)[0] + "/camera_info"
        assert derived == declared, (
            f"{name}: declares {declared} but Fortress derives {derived}; "
            "nest the image topic one level deeper"
        )


def test_every_camera_image_is_bridged():
    flat = bridge_arguments()
    for name, (image, _) in cameras().items():
        assert f"{image}@sensor_msgs/msg/Image" in flat, f"{name} image"


def test_only_the_thermal_info_comes_from_a_node():
    """Fortress publishes the wrong intrinsics for the thermal sensor.

    It fills CameraInfo from the default camera - fx 277 and centre (160, 120)
    for a 160 x 120 / 57 deg sensor - while RGB and depth report correctly.
    Bridging it would put a wrong CameraInfo on the topic that
    thermal_camera_info.py is there to provide, and last writer would win.
    """
    flat = bridge_arguments()
    for name, (_, info) in cameras().items():
        bridged = f"{info}@sensor_msgs/msg/CameraInfo" in flat
        if "thermal" in name:
            assert not bridged, "thermal camera_info must not be bridged from gz"
        else:
            assert bridged, f"{name} camera_info"


if __name__ == "__main__":
    test_cameras_exist()
    test_info_topics_are_unique()
    test_declared_info_topic_matches_the_derived_one()
    test_every_camera_image_is_bridged()
    test_only_the_thermal_info_comes_from_a_node()
    print(f"ok: {cameras()}")
