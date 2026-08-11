#!/usr/bin/env python3
"""Add a realistic random-walking person and dynamic heat to the demo world.

The source world and heat profile are never modified. A generated runtime
world receives configured equipment heat sources, low-cost transient diffusion
layers, and one complete factory worker with body heat and constrained random
movement.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


PERSON_TEMPERATURE_K = 309.15
AMBIENT_TEMPERATURE_K = 293.15
EFFECTIVE_DIFFUSIVITY_M2_S = 0.0018
DIFFUSION_RESPONSE_TIME_S = 3.5
DIFFUSION_UPDATE_PERIOD_S = 0.5
DIFFUSION_DISTANCE_MULTIPLIERS = (1.6, 3.0, 4.8)
DIFFUSION_BLOB_COUNT = 6


def thermal_plugin(temperature_k: float) -> str:
    return f"""
          <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">
            <temperature>{temperature_k:.2f}</temperature>
          </plugin>"""


def material(diffuse: str) -> str:
    return f"""
          <material>
            <ambient>{diffuse}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>"""


def equipment_core(source: dict[str, object]) -> str:
    radius = float(source["radius_m"])
    temperature_k = float(source["temperature_c"]) + 273.15
    return f"""
    <model name="{source['detection_id']}_core">
      <static>true</static>
      <pose>{source['x']} {source['y']} {source['z']} 0 0 0</pose>
      <link name="heat_link">
        <visual name="heat_visual">
          <geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>
          {material('0.36 0.12 0.08 1')}
          <cast_shadows>false</cast_shadows>{thermal_plugin(temperature_k)}
        </visual>
      </link>
    </model>"""


def diffusion_layer_model(
    source: dict[str, object], layer_index: int, distance: float
) -> str:
    source_radius = float(source["radius_m"])
    blob_radius = max(source_radius * 0.48, distance * 0.42)
    visuals = []
    for index in range(DIFFUSION_BLOB_COUNT):
        angle = 2.0 * math.pi * index / DIFFUSION_BLOB_COUNT
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        visuals.append(
            f"""
        <visual name="diffusion_{index}">
          <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
          <geometry><sphere><radius>{blob_radius:.4f}</radius></sphere></geometry>
          {material('0.25 0.18 0.12 1')}
          <cast_shadows>false</cast_shadows>
        </visual>"""
        )
    return f"""
    <model name="{source['detection_id']}_diffusion_{layer_index}">
      <static>true</static>
      <pose>0 0 -10 0 0 0</pose>
      <link name="heat_link">{''.join(visuals)}
      </link>
    </model>"""


def person_model() -> str:
    # Rounded centerline keeps the 0.53 m-wide mesh inside the 0.62 m aisle.
    plant_x = 2.35
    plant_y = 1.115
    radius = 0.31
    waypoints: list[tuple[float, float]] = []

    def append_point(x: float, y: float) -> None:
        if not waypoints or math.hypot(
            x - waypoints[-1][0], y - waypoints[-1][1]
        ) > 1e-6:
            waypoints.append((x, y))

    def append_line(x0: float, y0: float, x1: float, y1: float) -> None:
        distance = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, math.ceil(distance / 0.22))
        for step in range(steps + 1):
            ratio = step / steps
            append_point(x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio)

    def append_arc(cx: float, cy: float, start: float, end: float) -> None:
        for step in range(10):
            heading = start + (end - start) * step / 9
            append_point(
                cx + radius * math.sin(heading),
                cy - radius * math.cos(heading),
            )

    start_x = 0.10
    append_line(start_x, -plant_y - radius, plant_x, -plant_y - radius)
    append_arc(plant_x, -plant_y, 0.0, math.pi / 2.0)
    append_line(plant_x + radius, -plant_y, plant_x + radius, plant_y)
    append_arc(plant_x, plant_y, math.pi / 2.0, math.pi)
    append_line(plant_x, plant_y + radius, -plant_x, plant_y + radius)
    append_arc(-plant_x, plant_y, math.pi, 3.0 * math.pi / 2.0)
    append_line(-plant_x - radius, plant_y, -plant_x - radius, -plant_y)
    append_arc(-plant_x, -plant_y, 3.0 * math.pi / 2.0, 2.0 * math.pi)
    append_line(-plant_x, -plant_y - radius, start_x, -plant_y - radius)
    if math.hypot(
        waypoints[-1][0] - waypoints[0][0],
        waypoints[-1][1] - waypoints[0][1],
    ) < 1e-6:
        waypoints.pop()

    edges = [
        (index, (index + 1) % len(waypoints))
        for index in range(len(waypoints))
    ]
    waypoint_xml = "\n".join(
        f"        <waypoint>{x:.3f} {y:.3f}</waypoint>" for x, y in waypoints
    )
    edge_xml = "\n".join(
        f"        <edge>{start} {end}</edge>" for start, end in edges
    )
    return f"""
    <model name="thermal_factory_worker">
      <static>true</static>
      <pose>{start_x:.3f} {-plant_y - radius:.3f} 0 0 0 0</pose>
      <link name="person_link">
        <visual name="person_visual">
          <!-- Source mesh faces local +Y; rotate it so its face follows +X. -->
          <pose>0 0 0 0 0 -1.570796</pose>
          <geometry>
            <mesh>
              <uri>model://factory_worker_complete/meshes/worker_baked.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <cast_shadows>true</cast_shadows>{thermal_plugin(PERSON_TEMPERATURE_K)}
        </visual>
      </link>
      <plugin filename="hazard_guard_random_walk_system"
              name="hazard_guard_simulation::RandomWalkSystem">
        <speed>0.30</speed>
        <minimum_speed>0.22</minimum_speed>
        <maximum_speed>0.38</maximum_speed>
        <minimum_pause>0.25</minimum_pause>
        <maximum_pause>1.40</maximum_pause>
        <backtrack_probability>0.22</backtrack_probability>
        <arrival_tolerance>0.025</arrival_tolerance>
        <!-- zero selects a fresh random seed for each simulation run -->
        <seed>0</seed>
        <start_node>0</start_node>
{waypoint_xml}
{edge_xml}
      </plugin>
    </model>"""


def transfer_controller(sources: list[dict[str, object]]) -> str:
    layers = []
    for source in sources:
        radius = float(source["radius_m"])
        source_temperature_k = float(source["temperature_c"]) + 273.15
        decay_length = max(0.16, radius * 3.5)
        pose = f"{source['x']} {source['y']} {source['z']} 0 0 0"
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
                diffusion_layer_model(source, layer_index, radius * multiplier)
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
