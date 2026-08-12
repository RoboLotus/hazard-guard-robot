"""Every camera sensor must own its CameraInfo topic and be bridged.

Fortress builds the CameraInfo topic by dropping the last element of the image
topic and appending /camera_info, so two sensors publishing on /a and /b both
end up on /camera_info and overwrite each other's intrinsics. That failure is
silent - the bridge still runs and the topic still ticks - so it is checked
here instead of in the simulator.
"""
import re
import xml.etree.ElementTree as ElementTree
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
ROBOT = PACKAGE / "urdf" / "hazard_guard_m1.urdf.xacro"
LAUNCH = PACKAGE / "launch" / "simulation.launch.py"
CAMERA_TYPES = {"camera", "depth_camera", "thermal_camera"}


def camera_topics() -> dict:
    """sensor name -> published image topic, for every camera in the URDF."""
    root = ElementTree.parse(ROBOT).getroot()
    topics = {}
    for sensor in root.iter("sensor"):
        if sensor.get("type") not in CAMERA_TYPES:
            continue
        topic = sensor.find("topic")
        assert topic is not None, f"{sensor.get('name')} publishes no topic"
        topics[sensor.get("name")] = topic.text.strip()
    return topics


def info_topic(image_topic: str) -> str:
    return image_topic.rsplit("/", 1)[0] + "/camera_info"


def test_cameras_exist():
    assert len(camera_topics()) >= 2, "calibration needs two camera streams"


def test_info_topics_are_unique():
    infos = [info_topic(t) for t in camera_topics().values()]
    assert all(i != "/camera_info" for i in infos), (
        "a root-level image topic collapses its CameraInfo onto /camera_info"
    )
    assert len(set(infos)) == len(infos), f"CameraInfo topics collide: {infos}"


def test_every_camera_is_bridged():
    source = LAUNCH.read_text()
    # The bridge list wraps long entries across lines; join them back up.
    flat = re.sub(r'"\s*\n\s*"', "", source)
    for name, topic in camera_topics().items():
        assert f"{topic}@sensor_msgs/msg/Image" in flat, f"{name} image"
        assert (
            f"{info_topic(topic)}@sensor_msgs/msg/CameraInfo" in flat
        ), f"{name} camera_info"


if __name__ == "__main__":
    test_cameras_exist()
    test_info_topics_are_unique()
    test_every_camera_is_bridged()
    print(f"ok: {camera_topics()}")
