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


def test_capture_profile_uses_external_pose_without_rtabmap_loop_closures():
    params = yaml.safe_load(CAPTURE_PROFILE.read_text(encoding="utf-8"))[
        "/rtabmap/rtabmap"
    ]["ros__parameters"]

    assert params["publish_tf"] is False
    assert params["odom_frame_id"] == "map"
    assert params["map_frame_id"] == "map"
    assert params["subscribe_scan"] is False
    assert params["RGBD/NeighborLinkRefining"] == "false"
    assert params["RGBD/ProximityByTime"] == "false"
    assert params["RGBD/ProximityBySpace"] == "false"
    assert params["RGBD/ProximityPathMaxNeighbors"] == "0"
    assert params["RGBD/AggressiveLoopThr"] == "1.0"
    assert params["RGBD/OptimizeMaxError"] == "3.0"
    assert params["Rtabmap/LoopThr"] == "1.0"
    assert params["Grid/CellSize"] == "0.03"
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


def test_simulation_optimized_cloud_replaces_irreversible_raw_assembly():
    source = SIM_RTABMAP.read_text(encoding="utf-8")

    assert '"optimized_cloud"' in source
    assert 'executable="map_assembler"' in source
    assert '"Grid/CellSize": "0.03"' in source
    assert "UnlessCondition(LaunchConfiguration(\"optimized_cloud\"))" in source
    assert 'executable="adaptive_cloud_guard.py"' in source
    assert '"normal_points": 9000' in source
    assert '"/hazard_guard/rtabmap/cloud_surface_optimized"' in source
    assert '"/hazard_guard/rtabmap/cloud_surface"' in source


def test_physical_capture_reuses_field_tested_patrol_stack():
    wrapper = PHYSICAL_CAPTURE.read_text(encoding="utf-8")
    patrol = PHYSICAL_PATROL.read_text(encoding="utf-8")

    assert '"physical_patrol.launch.py"' in wrapper
    assert '"enable_rgbd_mapping": "true"' in wrapper
    assert '"use_person_safety": "false"' in wrapper
    assert '"person_device": ""' in wrapper
    assert '"enable_thermal_pipeline": "false"' in wrapper
    assert '"thermal_baseline_path": ""' in wrapper
    assert '"thermal_sensor_timeout_sec": "5.0"' in wrapper
    assert "*feature_defaults" in wrapper
    assert "**forwarded" in wrapper
    assert '"rgbd_capture_after_localization.launch.py"' in patrol
    assert '"backend": "real"' in patrol
    assert '"map": map_path' in patrol
    assert '"readiness_timeout_sec": "60.0"' in patrol


def test_physical_capture_has_safe_defaults_for_optional_path_selections():
    source = PHYSICAL_CAPTURE.read_text(encoding="utf-8")

    # A map is deliberately mandatory, but callers may omit optional DB and
    # storage selections. Explicit WebUI paths still override these defaults.
    assert 'DeclareLaunchArgument("map")' in source
    assert (
        '"rtabmap_database_path",\n'
        '                default_value="/tmp/hazard_guard_physical_rgbd.db"'
    ) in source
    assert (
        '"rtabmap_storage_path",\n'
        '                default_value="/tmp"'
    ) in source


def test_physical_second_pass_reset_reaches_only_selected_rtabmap_database():
    wrapper = PHYSICAL_CAPTURE.read_text(encoding="utf-8")
    patrol = PHYSICAL_PATROL.read_text(encoding="utf-8")
    gate = CAPTURE_GATE.read_text(encoding="utf-8")
    rtabmap = REAL_RTABMAP.read_text(encoding="utf-8")

    assert '"rtabmap_reset_database",\n                default_value="false"' in wrapper
    assert '"rtabmap_reset_database",' in wrapper.split("forwarded =", 1)[1]
    assert '"rtabmap_reset_database",\n                default_value="false"' in patrol
    assert '"reset_database": LaunchConfiguration(' in patrol
    assert '"rtabmap_reset_database"' in patrol
    assert '"reset_database": reset_database' in gate
    assert 'DeclareLaunchArgument(\n                "reset_database"' in rtabmap
    assert 'arguments=["-d"] if reset_database else []' in rtabmap
    assert 'IfCondition(LaunchConfiguration("reset_database"))' in rtabmap
    assert 'UnlessCondition(LaunchConfiguration("reset_database"))' in rtabmap


def test_capture_gate_starts_rtabmap_only_after_successful_readiness():
    source = CAPTURE_GATE.read_text(encoding="utf-8")

    assert 'executable="localization_ready_gate.py"' in source
    assert "OnProcessExit" in source
    assert "event.returncode != 0" in source
    assert "RTAB-Map was not started" in source
    assert '"odom_frame_id": "map"' in source
    assert '"map_frame_id": "map"' in source
    assert source.count('"optimized_cloud": "true"') == 2
    assert '"proximity_by_space": "false"' in source
    assert '"loop_closure_threshold": "1.0"' in source
    assert '"optimize_max_error": "3.0"' in source
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
