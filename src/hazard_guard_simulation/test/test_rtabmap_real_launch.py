from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


LAUNCH = (
    Path(__file__).parents[1] / "launch" / "rtabmap_real.launch.py"
)
PHYSICAL_MAPPING = (
    Path(__file__).parents[1] / "launch" / "physical_mapping.launch.py"
)
SPEC = spec_from_file_location("rtabmap_real_launch", LAUNCH)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_registration_strategy_has_supported_choices_and_safe_default():
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"rtabmap_registration_strategy"' in source
    assert 'choices=["0", "1", "2"]' in source
    assert 'default_value="1"' in source
    assert 'value_type=str' in source


def test_stamp_mode_choices_match_physical_entrypoint():
    source = LAUNCH.read_text(encoding="utf-8")
    physical_source = PHYSICAL_MAPPING.read_text(encoding="utf-8")
    choices = 'choices=["preserve", "offset", "latest"]'

    assert choices in source
    assert choices in physical_source


def test_cloud_frames_are_selectable_but_keep_odom_defaults():
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"cloud_fixed_frame"' in source
    assert '"cloud_output_frame"' in source
    # Cloud input/output and the external RTAB-Map pose frame each accept
    # odom/map. The latter preserves odom as its legacy default.
    assert source.count('choices=["odom", "map"]') == 3
    assert '"fixed_frame_id": LaunchConfiguration(' in source
    assert '"frame_id": LaunchConfiguration("cloud_output_frame")' in source


def test_optimized_map_is_opt_in_and_selected_on_the_public_cloud_path():
    source = LAUNCH.read_text(encoding="utf-8")

    optimized_default = (
        '"optimized_cloud",\n'
        '                default_value="false"'
    )
    assert optimized_default in source
    assert 'executable="map_assembler"' in source
    optimized_condition = (
        'condition=IfCondition(LaunchConfiguration("optimized_cloud"))'
    )
    assert optimized_condition in source
    assert '"/hazard_guard/rtabmap/cloud_surface_optimized"' in source
    assert '"/hazard_guard/rtabmap/cloud_surface"' in source
    assert '"/hazard_guard/rtabmap/cloud_frame_raw"' in source
    assert '"Grid/3D": "true"' in source
    assert 'surface_input_topic = PythonExpression(' in source
    assert source.count('"Grid/CellSize": ParameterValue(') == 2
    assert 'condition=UnlessCondition(optimized_cloud)' in source


def test_rtabmap_does_not_publish_a_second_parent_for_odom():
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"publish_tf": False' in source
    assert '"map_frame_id": map_frame_id' in source


def test_database_reset_is_explicit_opt_in_and_uses_rtabmap_delete_flag():
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"reset_database",\n                default_value="false"' in source
    assert 'arguments=["-d"] if reset_database else []' in source
    assert 'IfCondition(LaunchConfiguration("reset_database"))' in source
    assert 'UnlessCondition(LaunchConfiguration("reset_database"))' in source
    assert source.count("_rtabmap_node(") == 3


def test_external_pose_frame_is_configurable_with_legacy_default():
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"odom_frame_id"' in source
    assert 'default_value="odom"' in source
    assert '"odom_frame_id": odom_frame_id' in source


def test_map_frame_preserves_legacy_default_but_allows_saved_map_alignment():
    source = LAUNCH.read_text(encoding="utf-8")

    assert '"map_frame_id"' in source
    assert 'default_value="rtabmap_map"' in source
    assert 'choices=["rtabmap_map", "map"]' in source


@pytest.mark.parametrize(
    ("fixed_frame", "output_frame"),
    [("map", "map"), ("map", "odom"), ("odom", "map")],
)
def test_map_frame_rejects_latest_timestamp_policy(
    fixed_frame,
    output_frame,
):
    with pytest.raises(ValueError, match="cannot be combined"):
        MODULE.validate_cloud_configuration(
            fixed_frame,
            output_frame,
            "latest",
        )


@pytest.mark.parametrize("stamp_mode", ["preserve", "offset"])
def test_map_frame_accepts_timestamp_aware_policies(stamp_mode):
    MODULE.validate_cloud_configuration("map", "map", stamp_mode)


def test_default_odom_latest_combination_remains_compatible():
    MODULE.validate_cloud_configuration("odom", "odom", "latest")


def test_rtabmap_map_is_reserved_for_optimized_message_output():
    with pytest.raises(ValueError, match="must be"):
        MODULE.validate_cloud_configuration(
            "rtabmap_map",
            "rtabmap_map",
            "preserve",
        )


def test_physical_mapping_forwards_all_rtabmap_experiment_controls():
    source = PHYSICAL_MAPPING.read_text(encoding="utf-8")
    arguments = (
        "cloud_stamp_mode",
        "cloud_stamp_offset_sec",
        "sync_diagnostics",
        "rtabmap_registration_strategy",
        "cloud_fixed_frame",
        "cloud_output_frame",
        "optimized_cloud",
    )

    for argument in arguments:
        declaration = f'DeclareLaunchArgument(\n                "{argument}"'
        assert declaration in source
        assert f'"{argument}": LaunchConfiguration(' in source
    storage_declaration = (
        'DeclareLaunchArgument(\n'
        '                "storage_path"'
    )
    assert storage_declaration in source
    assert '"storage_path": storage_path' in source
    nested_arguments = (
        '"rtabmap_real.launch.py",\n'
        "                rtabmap_arguments"
    )
    assert nested_arguments in source
    assert 'default_value=str(workspace / "runtime" / "maps")' in source
