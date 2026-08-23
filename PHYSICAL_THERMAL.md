# Physical thermal inspection

The physical patrol launch keeps thermal analysis disabled by default so the
current Jetson/Nav2 stack can run before the thermal camera is installed.
Enable it only after the TmSDK ROS bridge publishes a temperature image,
CameraInfo, and a registered depth stream.

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

## Frozen-geometry cumulative thermal map

After the RGB-D mapping pass has exported a fixed `cloud.ply`, patrol can add
temperature attributes to that geometry without rebuilding or extending the
3D surface:

```bash
ros2 launch hazard_guard_simulation physical_patrol.launch.py \
  map:=/absolute/path/to/map.yaml \
  enable_frozen_thermal_map:=true \
  thermal_map_session_id:=facility-20260810-155822 \
  thermal_map_cloud_path:=/absolute/session/path/cloud.ply \
  thermal_map_state_path:=/absolute/session/path/thermal_layer.npz
```

`enable_frozen_thermal_map` also starts the existing live thermal-depth fusion
pipeline. The live `/hazard_guard/thermal/points` topic remains available for
current-frame analysis. The accumulator validates that input is already in the
`map` frame, matches only existing PLY voxels, and publishes a cumulative
snapshot on `/hazard_guard/thermal/map`. Its fields are `x`, `y`, `z`, `rgb`,
`temperature_c`, and `confidence`. Status is a transient-local JSON message on
`/hazard_guard/thermal/map/status`.

The default policy requires three stable localization samples, a timestamped
`map -> base_footprint` and `map -> thermal_camera_optical_frame` transform,
at least a 30% surface match ratio, an 8 cm Euclidean association, and at most
a 5 cm live/fixed range residual. A frame that fails any gate cannot modify
the thermal layer. Motion keyframes use 10 cm or 6 degrees; stationary
equipment is still refreshed every five seconds. The node never publishes TF
or changes AMCL/Nav2.

`thermal_layer.npz` is atomically checkpointed and includes the fixed geometry
fingerprint. A checkpoint from a different `cloud.ply` is rejected and is
never overwritten automatically. Unseen voxels retain their prior
temperature, count, confidence, and last-seen timestamp.
