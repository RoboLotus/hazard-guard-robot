# Physical thermal inspection

The physical patrol launch keeps thermal analysis disabled by default so the
current Jetson/Nav2 stack can run before the thermal camera is installed.
Enable it only after the TmSDK ROS bridge publishes a temperature image,
CameraInfo, and a registered depth stream.

## Map-bound equipment contract

Physical equipment is registered only after the same session has a saved 2D
map and a `map`-frame `cloud.ply`. The console stores `equipment.json` and
`route.json` beside that session under `runtime/maps/<world>/<session>/`.
Configuration schema 2 includes `world_id`, `map_session_id`, and
`frame_id=map`; the analyzer rejects another session or frame. A newly created
2D session intentionally starts with no equipment or route, so demo/legacy
`odom` ROIs are never activated automatically.

During patrol the frozen-map node publishes current samples that match the
immutable PLY on `/hazard_guard/thermal/static_observations`. The equipment
analyzer consumes this topic. Transient geometry is still available through
`/hazard_guard/thermal/dynamic` for visualization, but does not contribute to
fixed-equipment temperature statistics. Enabled equipment AABBs must not
overlap and retain at least 0.03 m clearance.

```bash
ros2 launch hazard_guard_simulation physical_patrol.launch.py \
  map:=/absolute/path/to/map.yaml \
  enable_thermal_pipeline:=true \
  thermal_roi_config:=/absolute/path/to/physical_rois.json \
  thermal_image_topic:=/tmsdk/thermal/image_raw \
  thermal_info_topic:=/tmsdk/thermal/camera_info \
  thermal_depth_image_topic:=/depth_camera/image_raw \
  thermal_depth_info_topic:=/depth_camera/camera_info \
  thermal_scale:=1.0 \
  thermal_offset_c:=0.0
```

The topic names above are examples. Set them to the topics exposed by the
actual TmSDK bridge. `thermal_scale` and `thermal_offset_c` must convert the
incoming pixels to Celsius; do not reuse the Gazebo Kelvin conversion without
checking the physical camera encoding.

The physical ROI file cannot reuse demo map coordinates. Survey each asset in
the physical `map` frame and assign the same `equipment_id` to its ROI and to
the inspection waypoint. A waypoint without `equipment_id` remains a normal
navigation/dwell point and does not collect an equipment-specific visit.

History is stored by default at:

```text
~/.local/share/hazard_guard/thermal_history.jsonl
```

## Immutable static map + persistent dynamic thermal layer

After the RGB-D mapping pass has exported `cloud.ply`, patrol keeps that file
as an immutable static base. Calibrated thermal RGB-D points update thermal
attributes on matching static voxels. A connected group that cannot be
explained by the static surface is tracked in a separate dynamic voxel layer:

```bash
ros2 launch hazard_guard_simulation physical_patrol.launch.py \
  map:=/absolute/path/to/map.yaml \
  enable_frozen_thermal_map:=true \
  thermal_map_session_id:=facility-20260810-155822 \
  thermal_map_cloud_path:=/absolute/session/path/cloud.ply \
  thermal_map_state_path:=/absolute/session/path/thermal_layer.npz \
  thermal_dynamic_state_path:=/absolute/session/path/dynamic_layer.npz
```

`enable_frozen_thermal_map` also starts the existing live thermal-depth fusion
pipeline. `/hazard_guard/thermal/points` is the current calibrated RGB-D cloud
in `map`; it contains actual depth geometry plus temperature and confidence.
The static base is never modified. Confirmed dynamic geometry is published on
`/hazard_guard/thermal/dynamic` with `temperature_c`, `confidence`,
`hit_count`, `miss_count`, and `last_seen_sec` fields. For the existing Console
contract,
`/hazard_guard/thermal/map` contains the static thermal attributes plus the
confirmed dynamic thermal voxels; deleting a dynamic voxel therefore reveals
the independently rendered static base again. Status is a transient-local JSON
message on `/hazard_guard/thermal/map/status`.

The default policy requires three stable localization samples, a timestamped
`map -> base_footprint` and `map -> thermal_camera_optical_frame` transform,
at least a 30% surface match ratio, an 8 cm Euclidean association, and at most
a 5 cm live/fixed range residual. A frame that fails any gate cannot modify
the static thermal layer. Dynamic candidates use 5 cm voxels, require a
connected component of at least eight voxels and two hits, and are removed
after three positive misses. A miss is counted only when a later depth ray
actually covers the old voxel range. A voxel outside the observed FoV, behind
a nearer return, or in a direction without valid depth keeps its state. Motion
keyframes use 10 cm or 6 degrees; stationary equipment is still refreshed
every five seconds. The node never publishes TF or changes AMCL/Nav2.

Both NPZ files are atomically checkpointed and include the fixed geometry
fingerprint. When `thermal_dynamic_state_path` is empty, the node derives a
`.dynamic.npz` sibling from `thermal_map_state_path`. A checkpoint from a
different `cloud.ply` is rejected and is never overwritten automatically.
Each dynamic voxel persists its point, temperature statistics, `last_seen`,
`hit_count`, `miss_count`, and confirmation state.
