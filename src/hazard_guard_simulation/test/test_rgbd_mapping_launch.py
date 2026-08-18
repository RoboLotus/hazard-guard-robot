from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
SIM_CAPTURE = PACKAGE / "launch" / "rgbd_mapping.launch.py"
PHYSICAL_CAPTURE = PACKAGE / "launch" / "physical_rgbd_mapping.launch.py"
LOCALIZATION = PACKAGE / "launch" / "localization.launch.py"
PHYSICAL_PATROL = PACKAGE / "launch" / "physical_patrol.launch.py"
SIM_RTABMAP = PACKAGE / "launch" / "rtabmap_sim.launch.py"
REAL_RTABMAP = PACKAGE / "launch" / "rtabmap_real.launch.py"
CAPTURE_GATE = (
    PACKAGE / "launch" / "rgbd_capture_after_localization.launch.py"
)
CAPTURE_PROFILE = PACKAGE / "config" / "rtabmap_rgbd_capture.yaml"


def test_simulation_capture_reuses_saved_map_localization():
    source = SIM_CAPTURE.read_text(encoding="utf-8")

    assert '"localization.launch.py"' in source
    assert '"enable_rgbd_mapping": "true"' in source
    assert '"map"' in source
    assert '"rtabmap_database_path"' in source


def test_capture_profile_uses_external_pose_without_duplicate_constraints():
    params = yaml.safe_load(CAPTURE_PROFILE.read_text(encoding="utf-8"))[
        "/rtabmap/rtabmap"
    ]["ros__parameters"]

    assert params["publish_tf"] is False
    assert params["odom_frame_id"] == "map"
    assert params["subscribe_scan"] is False
    assert params["RGBD/NeighborLinkRefining"] == "false"
    assert params["RGBD/ProximityBySpace"] == "false"
    assert params["Rtabmap/LoopThr"] == "0.0"
    assert params["Mem/IncrementalMemory"] == "true"


def test_localization_starts_capture_profile_only_when_requested():
    source = LOCALIZATION.read_text(encoding="utf-8")

    declaration = (
        'DeclareLaunchArgument("enable_rgbd_mapping", default_value="false")'
    )
    assert declaration in source
    assert '"rgbd_capture_after_localization.launch.py"' in source
    assert 'condition=IfCondition(enable_rgbd_mapping)' in source
    assert '"readiness_timeout_sec": "45.0"' in source


def test_simulation_rtabmap_resolves_runtime_parameter_file():
    source = SIM_RTABMAP.read_text(encoding="utf-8")

    assert "ParameterFile(parameters, allow_substs=True)" in source
    helper = source.split("def _rtabmap_node", 1)[1].split(
        "def _color_cloud_assembler_node", 1
    )[0]
    assert "str(parameters)" not in helper


def test_physical_capture_reuses_field_tested_patrol_stack():
    wrapper = PHYSICAL_CAPTURE.read_text(encoding="utf-8")
    patrol = PHYSICAL_PATROL.read_text(encoding="utf-8")

    assert '"physical_patrol.launch.py"' in wrapper
    assert '"enable_rgbd_mapping": "true"' in wrapper
    assert '"use_person_safety": "false"' in wrapper
    assert '"person_device": ""' in wrapper
    assert '"enable_thermal_pipeline": "false"' in wrapper
    assert "*feature_defaults" in wrapper
    assert "**forwarded" in wrapper
    assert '"rgbd_capture_after_localization.launch.py"' in patrol
    assert '"backend": "real"' in patrol
    assert '"map": map_path' in patrol
    assert '"readiness_timeout_sec": "60.0"' in patrol


def test_capture_gate_starts_rtabmap_only_after_successful_readiness():
    source = CAPTURE_GATE.read_text(encoding="utf-8")

    assert 'executable="localization_ready_gate.py"' in source
    assert "OnProcessExit" in source
    assert "event.returncode != 0" in source
    assert "RTAB-Map was not started" in source
    assert '"odom_frame_id": "map"' in source
    assert '"rtabmap_rgbd_capture.yaml"' in source


def test_physical_rtabmap_exposes_constraint_controls():
    source = REAL_RTABMAP.read_text(encoding="utf-8")

    for name in (
        "subscribe_scan",
        "neighbor_link_refining",
        "proximity_by_space",
        "loop_closure_threshold",
    ):
        assert f'"{name}"' in source
