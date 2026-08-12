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


def test_cameras_exist():
    assert len(cameras()) >= 2, "calibration needs two camera streams"


def test_info_topics_are_unique():
    infos = [info for _, info in cameras().values()]
    assert len(set(infos)) == len(infos), f"CameraInfo topics collide: {infos}"


def test_every_camera_is_bridged():
    source = LAUNCH.read_text()
    # The bridge list wraps long entries across lines; join them back up.
    flat = re.sub(r'"\s*\n\s*"', "", source)
    for name, (image, info) in cameras().items():
        assert f"{image}@sensor_msgs/msg/Image" in flat, f"{name} image"
        assert f"{info}@sensor_msgs/msg/CameraInfo" in flat, f"{name} camera_info"


if __name__ == "__main__":
    test_cameras_exist()
    test_info_topics_are_unique()
    test_every_camera_is_bridged()
    print(f"ok: {cameras()}")
