import math

from hazard_guard_mock_robot.perception import (
    heat_source_temperature,
    transform_planar_point,
    visible_heat_sources,
)


SOURCES = [
    {"detection_id": "front", "x": 3.0, "y": 0.0},
    {"detection_id": "side", "x": 0.0, "y": 3.0},
    {"detection_id": "far", "x": 6.0, "y": 0.0},
]


def test_only_sources_inside_fov_and_range_are_visible():
    visible = visible_heat_sources(0, 0, 0, SOURCES)
    assert [source["detection_id"] for source in visible] == ["front"]


def test_robot_yaw_rotates_the_thermal_sector():
    visible = visible_heat_sources(0, 0, math.pi / 2, SOURCES)
    assert [source["detection_id"] for source in visible] == ["side"]


def test_detection_confidence_stays_bounded():
    visible = visible_heat_sources(0, 0, 0, [SOURCES[0]])
    assert 0 <= visible[0]["confidence"] <= 1


def test_planar_transform_moves_world_point_into_slam_map():
    half_angle = math.pi / 4
    transformed = transform_planar_point(
        2.0,
        1.0,
        0.4,
        translation=(1.0, -2.0, 0.1),
        rotation=(0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
    )

    assert math.isclose(transformed[0], 0.0, abs_tol=1e-9)
    assert math.isclose(transformed[1], 0.0, abs_tol=1e-9)
    assert math.isclose(transformed[2], 0.5, abs_tol=1e-9)


def test_incident_temperature_only_overrides_matching_equipment():
    waste = {
        "source": "gazebo:bunker_waste_pile",
        "temperature_c": 25.0,
    }
    motor = {
        "source": "gazebo:primary_shredder_motor",
        "temperature_c": 84.6,
    }
    temperatures = {"bunker_waste_pile": 57.5}
    assert heat_source_temperature(waste, temperatures) == 57.5
    assert heat_source_temperature(motor, temperatures) == 84.6
