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
    assert len(layers) == len(MODULE.DIFFUSION_DISTANCE_MULTIPLIERS)
    distances = [float(layer.findtext("distance")) for layer in layers]
    assert distances == sorted(distances)
    assert controller.find(".//delay") is None

    core = root.find(".//model[@name='sim-hot-motor_surface_core']")
    assert core is not None
    assert core.find(".//sphere") is None
    core_submeshes = [
        node.text for node in core.findall(".//submesh/name")
    ]
    assert core_submeshes == ["shredder_motor.006"]
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
