#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/humble/setup.bash
fi

package_prefix="$(ros2 pkg prefix hazard_guard_simulation)"
package_share="${package_prefix}/share/hazard_guard_simulation"
runtime_dir="${HAZARD_GUARD_RUNTIME_DIR:-/tmp/hazard_guard_simulation}"
generated_world="${runtime_dir}/demo_facility_people_heat.sdf"
heat_profile="${package_share}/config/heat_sources/demo_facility_scaled.json"

mkdir -p "${runtime_dir}"
python3 "${package_prefix}/lib/hazard_guard_simulation/build_demo_people_heat_world.py" \
  "${package_share}/worlds/demo_facility_scaled.sdf" \
  "${heat_profile}" \
  "${generated_world}"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${package_prefix}/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${package_share}/models:${IGN_GAZEBO_RESOURCE_PATH:-}"

exec ros2 launch hazard_guard_simulation simulation.launch.py \
  gui:=false \
  world:="${generated_world}" \
  world_name:=demo_facility_people_heat \
  heat_source_profile:="${heat_profile}" \
  spawn_x:=0.0975 \
  spawn_y:=-1.4121 \
  spawn_z:=0.04 \
  spawn_yaw:=0.0 \
  simulation_mode:=kinematic \
  include_dispenser:=true \
  dispenser_mass:=0.05 \
  "$@"
