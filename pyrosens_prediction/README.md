# Pyrosens Prediction

`pyrosens_prediction` forecasts how an ignition point drifts when it is driven by the live wind and heat fields published by the `pyrosens_progress` stack.  
The package outputs step-by-step console telemetry and an RViz overlay that makes the predicted path, temperature and heat gradient easy to review.

---

## Data Flow

| Source (package) | Topic | Message | Purpose |
| --- | --- | --- | --- |
| Wind sensor (`pyrosens_progress`) | `/wind/point` | `geometry_msgs/PointStamped` | Position of each wind sample |
|  | `/wind/velocity` | `geometry_msgs/Vector3Stamped` | Wind vector at the sampled point |
| Thermal interpolation (`pyrosens_progress`) | `/heat/samples` | `sensor_msgs/PointCloud2` (`x,y,z,temp`) | Rolling heat samples |
| Ignition source | `point_topic` (default `point`) | `geometry_msgs/PointStamped` | Current fire anchor |
| Prediction node | `/prediction_markers` | `visualization_msgs/MarkerArray` | RViz trail, heat colours, gradient arrow |
| Prediction node | `/prediction` | `sensor_msgs/PointCloud2` | Placeholder for downstream consumers |

---

## Prediction Model

1. **Wind interpolation** - recent wind samples are paired by timestamp, retained for a configurable window and blended with Inverse Distance Weighting (IDW). If too few neighbours remain, the closest sample is used.
2. **Heat interpolation** - `/heat/samples` is treated as a scattered set of temperature points. The same IDW kernel estimates temperature at the predicted location and reconstructs a local gradient vector.
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
# From the workspace root (ensure only one pyrosens_progress variant is visible)
colcon build --packages-select pyrosens_progress pyrosens_prediction
source install/setup.bash   # repeat in every terminal

# Terminal 1 - simulators
ros2 run pyrosens_progress main_sim
# Terminal 2 - wind sensor and interpolation
ros2 run pyrosens_progress main_wind
# Terminal 3 - thermal sensor and interpolation
ros2 run pyrosens_progress main_thermal
# Terminal 4 - prediction node
ros2 run pyrosens_prediction prediction_main
# Terminal 5 (optional) - RViz
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

Override any parameter with `--ros-args -p name:=value` or a YAML file.

---

## Extending

- Populate the `/prediction` cloud or emit a `nav_msgs/Path` so planners can ingest the forecast directly.
- Swap IDW for trilinear interpolation when heat and wind data arrive on a structured grid.
- Add diagnostics when the predictor frequently falls back to single-neighbour mode.

Keep this document updated as the model evolves.
