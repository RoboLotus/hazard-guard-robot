#!/usr/bin/env python3
"""Add a stationary factory worker and surface heat to the demo world.

The source world and heat profile are never modified. A generated runtime
world receives configured equipment heat sources, low-cost transient surface
zones, and one stationary factory worker with body heat. Thermal zones reuse
real equipment OBJ submeshes, so the thermal
camera sees heated machinery surfaces instead of floating diffusion spheres.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


PERSON_TEMPERATURE_K = 309.15
# The hall shell is about 1.22 m high after its world scale is applied. The
# source worker mesh is 1.856 m high, so 0.33 makes it roughly half as tall as
# the outside wall (about 0.61 m) for this compact simulation world.
PERSON_SCALE = 0.33
PERSON_POSE = (0.10, -1.425, 0.0, 0.0, 0.0, 0.0)
AMBIENT_TEMPERATURE_K = 293.15
EFFECTIVE_DIFFUSIVITY_M2_S = 0.0018
DIFFUSION_RESPONSE_TIME_S = 3.5
DIFFUSION_UPDATE_PERIOD_S = 0.5
DIFFUSION_DISTANCE_MULTIPLIERS = (1.6, 3.0, 4.8)
EQUIPMENT_MODEL_POSE = (-0.145177, -0.038474, 0.0, 0.0, 0.0, 0.0)
EQUIPMENT_MESH_SCALE = 0.07474982
# A 0.15% expansion prevents z-fighting while remaining within 3 mm of the
# original equipment surface at the largest model extents.
THERMAL_OVERLAY_SCALE = EQUIPMENT_MESH_SCALE * 1.0015

# Each temperature zone reuses submeshes from the existing factory equipment.
# The core is held at the configured source temperature. The three surrounding
# groups warm progressively through HeatTransferSystem.
SURFACE_HEAT_ZONES: dict[str, dict[str, object]] = {
    "sim-hot-motor": {
        "mesh_uri": "model://primary_shredder/meshes/primary_shredder.obj",
        "core": ("shredder_motor.006",),
        "layers": (
            ("discharge_conveyor_drum.012",),
            (
                "discharge_conveyor_side.013",
                "discharge_conveyor_frame.006",
                "shredder_body.006",
            ),
            ("shredder_base.006", "shredder_housing.006"),
        ),
    },
    "sim-pump-block": {
        "mesh_uri": (
            "model://secondary_processor/meshes/secondary_processor.obj"
        ),
        "core": ("sec_drive.006",),
        "layers": (
            ("sec_body.006",),
            ("sec_base.006", "sec_cover.006"),
            ("sec_hopper.006",),
        ),
    },
    "sim-tank-block": {
        "mesh_uri": "model://baler/meshes/baler.obj",
        "core": ("baler_cabinet.006",),
        "layers": (
            ("baler_infeed_leg.040", "baler_base.006"),
            ("baler_body.006", "baler_outfeed_table.006"),
            (
                "baler_press_head.006",
                "baler_infeed_side.012",
                "baler_infeed_side.013",
            ),
        ),
    },
    "sim-waste-pile": {
        "mesh_uri": "model://bunker/meshes/bunker.obj",
        "core": ("waste_chunk.1940",),
        "layers": (
            (
                "waste_chunk.1814",
                "waste_chunk.2037",
                "waste_chunk.2058",
                "waste_chunk.1837",
            ),
            (
                "waste_chunk.1827",
                "waste_chunk.2088",
                "waste_chunk.1862",
                "waste_chunk.1938",
                "waste_chunk.1982",
            ),
            (
                "waste_chunk.2076",
                "waste_chunk.2048",
                "waste_chunk.2081",
                "waste_chunk.1869",
                "waste_chunk.2023",
                "waste_chunk.1811",
            ),
        ),
    },
}


def thermal_plugin(temperature_k: float) -> str:
    return f"""
          <plugin filename="ignition-gazebo-thermal-system" name="ignition::gazebo::systems::Thermal">
            <temperature>{temperature_k:.2f}</temperature>
          </plugin>"""


def surface_profile(
    source: dict[str, object],
) -> dict[str, object]:
    detection_id = str(source["detection_id"])
    try:
        return SURFACE_HEAT_ZONES[detection_id]
    except KeyError as error:
        raise ValueError(
            f"No equipment surface profile for heat source {detection_id!r}"
        ) from error


def surface_visual(
    profile: dict[str, object],
    visual_name: str,
    submesh_name: str,
    temperature_k: float | None = None,
) -> str:
    temperature_xml = (
        thermal_plugin(temperature_k) if temperature_k is not None else ""
    )
    return f"""
        <visual name="{visual_name}">
          <geometry>
            <mesh>
              <uri>{profile['mesh_uri']}</uri>
              <scale>{THERMAL_OVERLAY_SCALE:.8f} {THERMAL_OVERLAY_SCALE:.8f} {THERMAL_OVERLAY_SCALE:.8f}</scale>
              <submesh>
                <name>{submesh_name}</name>
                <center>false</center>
              </submesh>
            </mesh>
          </geometry>
          <cast_shadows>false</cast_shadows>{temperature_xml}
        </visual>"""


def equipment_core(source: dict[str, object]) -> str:
    profile = surface_profile(source)
    temperature_k = float(source["temperature_c"]) + 273.15
    visuals = "".join(
        surface_visual(profile, f"core_surface_{index}", submesh, temperature_k)
        for index, submesh in enumerate(profile["core"])
    )
    pose = " ".join(f"{value:.6f}" for value in EQUIPMENT_MODEL_POSE)
    return f"""
    <model name="{source['detection_id']}_surface_core">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="heat_link">{visuals}
      </link>
    </model>"""


def surface_layer_model(
    source: dict[str, object], layer_index: int, distance: float
) -> str:
    profile = surface_profile(source)
    submeshes = profile["layers"][layer_index - 1]
    source_temperature_k = float(source["temperature_c"]) + 273.15
    radius = float(source["radius_m"])
    decay_length = max(0.16, radius * 3.5)
    temperature_rise = max(
        0.0, source_temperature_k - AMBIENT_TEMPERATURE_K
    ) * math.exp(-distance / decay_length)
    surface_temperature_k = AMBIENT_TEMPERATURE_K + temperature_rise
    visuals = "".join(
        surface_visual(
            profile, f"surface_{index}", submesh, surface_temperature_k
        )
        for index, submesh in enumerate(submeshes)
    )
    return f"""
    <model name="{source['detection_id']}_diffusion_{layer_index}">
      <static>true</static>
      <pose>0 0 -10 0 0 0</pose>
      <link name="heat_link">{visuals}
      </link>
    </model>"""


def person_model() -> str:
    pose = " ".join(f"{value:.3f}" for value in PERSON_POSE)
    return f"""
    <model name="thermal_factory_worker">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="person_link">
        <visual name="person_visual">
          <!-- Source mesh faces local +Y; rotate it so its face follows +X. -->
          <pose>0 0 0 0 0 -1.570796</pose>
          <geometry>
            <mesh>
              <uri>model://factory_worker_complete/meshes/worker_baked.obj</uri>
              <scale>{PERSON_SCALE} {PERSON_SCALE} {PERSON_SCALE}</scale>
            </mesh>
          </geometry>
          <cast_shadows>true</cast_shadows>{thermal_plugin(PERSON_TEMPERATURE_K)}
        </visual>
      </link>
    </model>"""


def transfer_controller(sources: list[dict[str, object]]) -> str:
    layers = []
    for source in sources:
        radius = float(source["radius_m"])
        source_temperature_k = float(source["temperature_c"]) + 273.15
        decay_length = max(0.16, radius * 3.5)
        pose = " ".join(f"{value:.6f}" for value in EQUIPMENT_MODEL_POSE)
        for layer_index, multiplier in enumerate(
            DIFFUSION_DISTANCE_MULTIPLIERS, start=1
        ):
            distance = radius * multiplier
            layers.append(
                f"""
        <layer>
          <model>{source['detection_id']}_diffusion_{layer_index}</model>
          <distance>{distance:.5f}</distance>
          <source_temperature>{source_temperature_k:.2f}</source_temperature>
          <decay_length>{decay_length:.5f}</decay_length>
          <pose>{pose}</pose>
        </layer>"""
            )
    return f"""
    <model name="heat_transfer_controller">
      <static>true</static>
      <pose>0 0 -10 0 0 0</pose>
      <link name="controller_link" />
      <plugin filename="hazard_guard_heat_transfer_system"
              name="hazard_guard_simulation::HeatTransferSystem">
        <ambient_temperature>{AMBIENT_TEMPERATURE_K:.2f}</ambient_temperature>
        <effective_diffusivity>{EFFECTIVE_DIFFUSIVITY_M2_S:.6f}</effective_diffusivity>
        <response_time>{DIFFUSION_RESPONSE_TIME_S:.2f}</response_time>
        <update_period>{DIFFUSION_UPDATE_PERIOD_S:.2f}</update_period>
        <minimum_visible_rise>0.35</minimum_visible_rise>{''.join(layers)}
      </plugin>
    </model>"""


def additions(profile: dict[str, object]) -> str:
    sources = list(profile["sources"])
    parts = [
        "\n    <!-- BEGIN HAZARD GUARD PEOPLE AND HEAT TRANSFER -->",
        person_model(),
    ]
    for source in sources:
        radius = float(source["radius_m"])
        parts.append(equipment_core(source))
        for layer_index, multiplier in enumerate(
            DIFFUSION_DISTANCE_MULTIPLIERS, start=1
        ):
            parts.append(
                surface_layer_model(source, layer_index, radius * multiplier)
            )
    parts.append(transfer_controller(sources))
    parts.append("    <!-- END HAZARD GUARD PEOPLE AND HEAT TRANSFER -->\n")
    return "\n".join(parts)


def build_world(source: Path, profile_path: Path, destination: Path) -> None:
    text = source.read_text(encoding="utf-8")
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    source_world = str(profile["world_id"])
    destination_world = "demo_facility_people_heat"
    opening = f'<world name="{source_world}">'
    closing = "  </world>"
    if text.count(opening) != 1 or text.count(closing) != 1:
        raise ValueError(f"Unexpected world structure in {source}")
    text = text.replace(opening, f'<world name="{destination_world}">', 1)
    text = text.replace(closing, additions(profile) + closing, 1)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("profile", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    build_world(args.source, args.profile, args.destination)
    print(args.destination)


if __name__ == "__main__":
    main()
