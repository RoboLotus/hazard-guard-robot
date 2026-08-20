from pathlib import Path

from hazard_guard_bag_recorder.profiles import (
    ProfileError,
    load_profile_document,
    resolve_profile,
)


PACKAGE = Path(__file__).resolve().parent.parent


def definitions():
    return load_profile_document(PACKAGE / "config" / "profiles.json")


def test_navigation_profile_is_explicit_and_excludes_yolo_topics():
    profile = resolve_profile("navigation-core", definitions())
    topics = {candidate for item in profile.topics for candidate in item.candidates}

    assert {"/tf", "/tf_static", "/odom", "/scan"}.issubset(topics)
    assert not any("person" in topic or "yolo" in topic for topic in topics)


def test_rgbd_profile_inherits_navigation_and_accepts_driver_aliases():
    profile = resolve_profile("rgbd-mapping", definitions())
    by_id = {item.identifier: item for item in profile.topics}

    assert by_id["scan"].required
    assert "/ascamera_hp60c/camera_publisher/depth0/image_raw" in by_id["depth_image"].candidates


def test_experimental_thermal_profiles_are_marked_for_runtime_confirmation():
    assert resolve_profile("thermal-calibration", definitions()).experimental
    assert resolve_profile("patrol-thermal", definitions()).experimental


def test_profile_cycle_and_unsafe_topic_are_rejected():
    cyclic = {"a-profile": {"extends": "b-profile"}, "b-profile": {"extends": "a-profile"}}
    try:
        resolve_profile("a-profile", cyclic)
    except ProfileError as exc:
        assert "cycle" in str(exc)
    else:
        raise AssertionError("inheritance cycle must fail")

    unsafe = {"safe-profile": {"topics": [{"id": "bad", "candidates": ["/scan; rm -rf /"]}]}}
    try:
        resolve_profile("safe-profile", unsafe)
    except ProfileError as exc:
        assert "invalid ROS topic" in str(exc)
    else:
        raise AssertionError("unsafe topic must fail")
