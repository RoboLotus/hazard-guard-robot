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
