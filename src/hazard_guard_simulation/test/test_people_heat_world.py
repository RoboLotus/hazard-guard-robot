import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET


SCRIPT = (
    Path(__file__).parents[1] / "scripts" / "build_demo_people_heat_world.py"
)
SPEC = importlib.util.spec_from_file_location("people_heat_world", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_generated_world_uses_transient_diffusion_and_stationary_worker(
    tmp_path,
):
    source = tmp_path / "source.sdf"
    profile = tmp_path / "profile.json"
    destination = tmp_path / "generated.sdf"
    source.write_text(
        '<sdf version="1.8"><world name="demo_facility_scaled">\n'
        '  </world></sdf>',
        encoding="utf-8",
    )
    profile.write_text(
        json.dumps(
            {
                "world_id": "demo_facility_scaled",
                "sources": [
                    {
                        "detection_id": "sim-hot-motor",
                        "x": 1.0,
                        "y": 2.0,
                        "z": 0.3,
                        "temperature_c": 80.0,
                        "radius_m": 0.05,
                        "incident_model": "discarded_lithium_power_bank",
                        "incident_model_yaw_rad": 0.35,
                        "temperature_topic": (
                            "/hazard_guard/incident/battery/temperature"
                        ),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    MODULE.build_world(source, profile, destination)
    root = ET.parse(destination).getroot()
    controller = root.find(".//plugin[@name='hazard_guard_simulation::HeatTransferSystem']")
    assert controller is not None
    assert float(controller.findtext("effective_diffusivity")) > 0.0
    assert float(controller.findtext("update_period")) == 0.5
    layers = controller.findall("layer")
    assert len(layers) == len(MODULE.DIFFUSION_DISTANCE_MULTIPLIERS) + 2
    distances = [float(layer.findtext("distance")) for layer in layers]
    assert distances == sorted(distances)
    assert layers[0].findtext("always_visible") == "true"
    assert all(
        layer.findtext("temperature_topic")
        == "/hazard_guard/incident/battery/temperature"
        for layer in layers
    )
    assert controller.find(".//delay") is None

    battery = root.find(".//model[@name='discarded_lithium_power_bank']")
    assert battery is not None
    battery_pose = [float(value) for value in battery.findtext("pose").split()]
    assert battery_pose[:2] == [1.0, 2.0]
    assert battery_pose[2] == 0.3 + MODULE.BATTERY_Z_OFFSET_M
    assert battery.find(".//sphere") is None
    assert battery.findtext(".//collision/geometry/box/size") == "0.090 0.045 0.018"
    visual_names = {
        visual.get("name") for visual in battery.findall(".//visual")
    }
    assert visual_names == {
        "body_visual",
        "top_label_visual",
        "usb_port_visual",
        "warning_band_visual",
    }
    assert len(battery.findall(".//plugin[@filename='ignition-gazebo-thermal-system']")) == 4
    controlled_models = [layer.findtext("model") for layer in layers]
    assert controlled_models[0] == "discarded_lithium_power_bank"

    core = root.find(".//model[@name='sim-hot-motor_surface_core']")
    assert core is not None
    assert core.find(".//sphere") is None
    core_submeshes = [
        node.text for node in core.findall(".//submesh/name")
    ]
    assert core_submeshes == ["shredder_motor.006"]
    core_thermal = core.find(".//plugin")
    assert core_thermal is not None
    assert core_thermal.get("filename") == "ignition-gazebo-thermal-system"
    assert (
        core_thermal.get("name") == "ignition::gazebo::systems::Thermal"
    )
    surface_layers = [
        root.find(f".//model[@name='sim-hot-motor_diffusion_{index}']")
        for index in range(1, len(MODULE.DIFFUSION_DISTANCE_MULTIPLIERS) + 1)
    ]
    assert all(layer is not None for layer in surface_layers)
    assert all(layer.find(".//sphere") is None for layer in surface_layers)
    assert all(layer.findall(".//submesh/name") for layer in surface_layers)
    layer_temperatures = [
        float(layer.findtext(".//temperature")) for layer in surface_layers
    ]
    assert layer_temperatures == sorted(layer_temperatures, reverse=True)
    assert layer_temperatures[0] < 80.0 + 273.15
    assert layer_temperatures[-1] > MODULE.AMBIENT_TEMPERATURE_K
    assert all(layer.findall(".//temperature") for layer in surface_layers)
    assert root.findall(".//sphere") == []

    worker = root.find(".//model[@name='thermal_factory_worker']")
    assert worker is not None
    assert worker.findtext("static") == "true"
    walker = worker.find(
        ".//plugin[@name='hazard_guard_simulation::RandomWalkSystem']"
    )
    assert walker is None
    scale = [float(value) for value in worker.findtext(".//mesh/scale").split()]
    assert scale == [MODULE.PERSON_SCALE] * 3
    pose = [float(value) for value in worker.findtext("pose").split()]
    assert pose == list(MODULE.PERSON_POSE)


def test_physical_patrol_exposes_opt_in_thermal_pipeline():
    launch_path = (
        Path(__file__).parents[1] / "launch" / "physical_patrol.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")

    assert '"enable_thermal_pipeline", default_value="false"' in source
    assert '"thermal_roi_config", default_value=""' in source
    assert '"simulated": "false"' in source
    assert '"use_sim_time": "false"' in source
    for argument in (
        "thermal_image_topic",
        "thermal_info_topic",
        "thermal_depth_image_topic",
        "thermal_depth_info_topic",
        "thermal_history_path",
    ):
        assert source.count(f'"{argument}"') >= 2
