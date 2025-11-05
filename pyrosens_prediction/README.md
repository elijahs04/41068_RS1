# Pyrosens Prediction

`pyrosens_prediction` forecasts how an ignition point drifts when it is driven by the live wind and heat fields published by the `pyrosens_integrated_sensors` stack.  
The package outputs step-by-step console telemetry and an RViz overlay that makes the predicted path, temperature and heat gradient easy to review.

---

## Data Flow

| Source (package) | Topic | Message | Purpose |
| --- | --- | --- | --- |
| Wind sensor (`pyrosens_integrated_sensors`) | `/wind/point` | `geometry_msgs/PointStamped` | Position of each wind sample |
|  | `/wind/velocity` | `geometry_msgs/Vector3Stamped` | Wind vector at the sampled point |
| Thermal hotspots (`pyrosens_integrated_sensors`) | `/hotspots/points_cloud` | `sensor_msgs/PointCloud2` (`x,y,z` + optional `temp`) | Rolling hotspot samples for interpolation |
| Ignition source | `point_topic` (default `point`) | `geometry_msgs/PointStamped` | Current fire anchor |
| Prediction node | `/prediction_markers` | `visualization_msgs/MarkerArray` | RViz trail, heat colours, gradient arrow |
| Prediction node | `/prediction` | `sensor_msgs/PointCloud2` | Placeholder for downstream consumers |

---

## Prediction Model

1. **Wind interpolation** - recent wind samples are paired by timestamp, retained for a configurable window and blended with Inverse Distance Weighting (IDW). If too few neighbours remain, the closest sample is used.
2. **Heat interpolation** - `/hotspots/points_cloud` (or any configured cloud) is treated as a scattered set of temperature points. The same IDW kernel estimates temperature at the predicted location and reconstructs a local gradient vector, falling back to a configurable default when no heat field is present.
3. **State propagation** - the ignition point is advanced in `predict_step` increments until `predict_time` is reached, using the interpolated wind vector for each step.
4. **Telemetry** - each iteration logs the new position, wind magnitude, neighbour counts, temperature estimate and gradient strength so you can cross-check the forecast.

---

## RViz Visualisation

The `/prediction_markers` stream contains:

- **Line strip** of the predicted trajectory.
- **Sphere list** coloured by temperature (blue -> cyan -> green -> yellow -> red).
- **Head sphere** placed at the final predicted point, using the same heat colour.
- **Gradient arrow** that points toward the steepest heat increase at the head (scale controlled by `gradient_arrow_scale`).

Add a *MarkerArray* display in RViz, select `/prediction_markers`, and keep the fixed frame set to `map`.

---

## Build & Run

```bash
# refer to the README for ignition bringup, launch drone with test world and no mapping before proceeding with the following
# From the workspace root
colcon build --packages-select pyrosens_integrated_sensors pyrosens_prediction
source install/setup.bash   # repeat in every terminal

# Terminal 1 – wind + thermal sensors
ros2 launch pyrosens_integrated_sensors integrated_sensors.launch.py

# Terminal 2 – prediction (manual point entry)
ros2 run pyrosens_prediction prediction_main --ros-args -p prediction_sim.point_mode:=manual

# Terminal 3 (optional) – RViz
rviz2
```

---

## Key Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `predict_time` | `10.0` | Total horizon (seconds) for each prediction cycle |
| `predict_step` | `1.0` | Integration time step |
| `wind_sample_retention_sec` | `5.0` | Keep wind samples for this many seconds |
| `wind_idw_radius` | `2.0` | Maximum radius (m) for wind neighbours (`0` disables the limit) |
| `heat_idw_radius` | `2.5` | Maximum radius (m) for heat neighbours |
| `idw_power` | `2.0` | IDW exponent for wind and heat |
| `idw_min_neighbors` | `3` | Minimum neighbours before falling back to the closest sample |
| `gradient_arrow_scale` | `0.5` | Arrow length multiplier for the heat gradient visual |
| `heat_color_min` / `heat_color_max` | `25.0` / `600.0` | Temperature range used for colour mapping |
| `wind_point_topic`, `wind_velocity_topic`, `heat_samples_topic` | see defaults | Override when wiring into a different stack |
| `heat_value_field` | `"temp"` | Preferred field name in the heat cloud; falls back through `temperature`, `intensity`, `i` |
| `default_heat_temperature` | `200.0` | Temperature used when the cloud does not provide a heat/intensity value |

Override any parameter with `--ros-args -p name:=value` (or target a specific node with `prediction_node.*` / `prediction_sim.*`) or supply a YAML file.

### PredictionSim parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `point_mode` | `manual` | Choose `manual`, `cloud`, or `disabled`; address as `prediction_sim.point_mode` when launching `prediction_main` |
| `point_cloud_topic` | `/hotspots/points_cloud` | Cloud used for centroid extraction when `point_mode=cloud` |
| `manual_point_frame` | `map` | Frame attached to manually entered points |
| `enable_fake_sensors` | `false` | Toggle the legacy synthetic wind/heat/point publishers |

---

## Extending

- Populate the `/prediction` cloud or emit a `nav_msgs/Path` so planners can ingest the forecast directly.
- Swap IDW for trilinear interpolation when heat and wind data arrive on a structured grid.
- Add diagnostics when the predictor frequently falls back to single-neighbour mode.

When `point_mode` is set to `cloud`, each incoming cloud is reduced to a centroid `PointStamped`, allowing the prediction node to follow whichever hotspot cluster the thermal stack is currently observing. The default `manual` mode keeps a console listener alive in `prediction_main`; type `x y z` and press Enter to publish a new ignition point, or `q` to stop manual publishing.

Keep this document updated as the model evolves.
