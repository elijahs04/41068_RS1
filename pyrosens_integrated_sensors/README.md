# PyroSens Integrated Sensors

This ROS 2 package bundles the wind-sensing pipeline that is currently fed by simulated telemetry and the thermal hotspot workflow that processes live Gazebo sensor feeds.  The launch file `integrated_sensors.launch.py` brings every node online so the system can publish a cohesive situational picture for downstream autonomy stacks, mapping, or RViz visualisation.

The sections below describe how the stack is wired together, which topics are exchanged, and how each node transforms its inputs step by step.  The workflow walkthroughs are intentionally detailed so you can explain what the code is doing without having to dive back into the sources.

---

## Quick Start

The thermal pipeline depends on the PyroSens Gazebo world so the simulated UAV can stream a real thermal camera and depth image.  Wind sensing does **not** rely on Gazebo—it is currently driven by a configurable internal simulator.

```bash
# Terminal 1 – bring up the Gazebo world that publishes the thermal/depth topics
cd ~/ros2_ws
colcon build --packages-select pyrosens_ignition_bringup pyrosens_mission
source ~/ros2_ws/install/setup.bash
ros2 launch pyrosens_ignition_bringup pyrosens_drone.launch.py slam:=true nav2:=true rviz:=true world:=multi_fire

# Terminal 2 – build and launch the integrated PyroSens stack
cd ~/ros2_ws
colcon build --packages-select pyrosens_integrated_sensors
source ~/ros2_ws/install/setup.bash
ros2 launch pyrosens_integrated_sensors integrated_sensors.launch.py
```

When RViz is open, add the following displays to see the outputs:

- `Image` display subscribed to `/thermal_hotspot/thermal_overlay`.
- `PointCloud2` display subscribed to `/hotspots/points_cloud`.
- `OccupancyGrid` display subscribed to `/hotspots/heatmap`.
- `MarkerArray` display subscribed to `/wind/markers`.

---

## High-Level Architecture

- **Thermal hotspot workflow (real sensing):**
  - `ThermalHotspotNode` ingests the real thermal and depth streams from Gazebo, thresholds hot regions, and reprojects them into the map frame.
  - `HotspotMapperNode` aggregates those 3D hotspots into an exponentially-decaying 2D heatmap for long-lived situational awareness.

- **Wind workflow (simulated sensing):**
  - `WindSimNode` produces a synthetic wind field as a point cloud with per-point velocity vectors.
  - `WindSensNode` mimics a scanning sensor by replaying each simulated sample as stamped point/velocity messages.
  - `WindInterpolationNode` pairs the position/velocity samples and draws coloured arrows in RViz.

All nodes are built into a shared library (`pyrosens_integrated_sensors_lib`) so you can re-use the core classes inside other executables if needed.

---

## Thermal Hotspot Workflow

`ThermalHotspotNode` (`src/thermalHotspotNode.cpp`) is where the real thermal camera is processed.  The node wires together OpenCV-based detection, TF transforms, and point cloud publishing.  The callback executed for every thermal+depth pair performs the following sequence:

1. **Synchronise inputs.** Two `message_filters::Subscriber` instances listen to the thermal camera (`/thermal_camera/image`) and depth camera (`/camera/depth/image`).  An approximate-time synchroniser ensures that callbacks receive pairs of images captured at nearly the same instant.
2. **Prepare intrinsics.** On the first callback the node computes pinhole intrinsics from the configured horizontal field of view (`hfov_rad` for thermal, `depth_hfov_rad` for depth).  These values are cached and reused for projection.
3. **Convert thermal image.** `cv_bridge::toCvShare` converts the ROS image into a `cv::Mat`.  The image must be `mono8` (Gazebo publishes 8-bit) and is left untouched otherwise.
4. **Build a hot-pixel mask.**
   - If `use_percentile` is `true`, `HotspotDetector::percentileRawMono8` finds the raw value that corresponds to the `hot_percentile` hottest pixels and uses that as the threshold.
   - Otherwise the configured `hot_temp_c` is converted back into the camera’s raw scale using `temp_gain` and `temp_offset`.
   - Optional morphological open/close (kernel radius `morphology_kernel`) removes speckles and fills gaps.
5. **Convert the depth image** to floating-point metres (`TYPE_32FC1`) or centimetres (`TYPE_16UC1`).  Pixels outside `[depth_min, depth_max]` are ignored.
6. **Look up transforms.** TF2 is queried for:
   - `thermal_frame <- depth_frame`: aligns depth points with the thermal camera.
   - `target_frame <- depth_frame`: usually `map`, used to express output hotspots in world coordinates.
7. **Iterate depth pixels.** With a stride (`depth_stride`) to reduce load, each valid depth sample is:
   - Back-projected into a 3D point in the depth optical frame.
   - Converted into the thermal camera frame using the TF transform.
   - Projected into the thermal image; if the corresponding pixel is “hot”, the 3D point is kept.
8. **Publish hotspot geometry.**
   - A `geometry_msgs::msg::PointStamped` (`hotspots/points_stamped`) for each hit, flattened to the map plane (`z = plane_z`).
   - A `sensor_msgs::msg::PointCloud2` (`hotspots/points_cloud`) containing all hotspots in the target frame.
   - A `geometry_msgs::msg::PoseArray` (`hotspots/world_points`) with centroid poses derived from `HotspotDetector::detectHotspots`.  Centroids are projected to map coordinates through `HotspotTransform::pixelToWorldXY`.
9. **Draw visual overlay.** When `publish_overlay` is enabled, the node draws bounding boxes around each detected region and republishes the RGB overlay as `/thermal_hotspot/thermal_overlay`.

> **Gazebo Integration:** The “real” sensing claim comes from the fact that the thermal and depth topics originate from the Gazebo simulation (not the wind simulator).  As long as the Gazebo launch is active, you are seeing the genuine sensor models respond to the environment and fire sources.

### Aggregating hotspots into a heatmap

`HotspotMapperNode` (`src/hotspotMapperNode.cpp`) listens to the outputs from the step above and maintains a 2D occupancy-style grid that slowly decays:

1. **Inputs.** It subscribes to `hotspots/points_stamped` and `hotspots/points_cloud`.  Either a single point or a full point cloud can update the map.
2. **Gaussian splatting.** Every point is converted into map indices (`worldToCell`).  A pre-built Gaussian kernel (variance `sigma_m`) spreads the contribution over neighbouring cells, increasing the accumulated weight (`add_weight`) in `grid_` and marking them as `seen`.
3. **Temporal decay.** A wall timer ticks at `decay_rate_hz`.  Each tick multiplies every cell by an exponential decay factor derived from `decay_half_life_s`, letting old hotspots fade out unless they are revisited.
4. **Thresholding.** Cells that fall below `clear_threshold` are emptied.  If `publish_unknown_as_unseen` is true they are also marked as unseen so the occupancy grid reports them as `-1` (unknown).
5. **Publishing outputs.**
   - `nav_msgs::msg::OccupancyGrid` on `/hotspots/heatmap` with intensities normalised to the 90th percentile, so outliers do not saturate the view.
   - `sensor_msgs::msg::Image` on `/hotspots/heatmap_image`, a false-colour Jet heatmap for RViz overlays.

Because this mapper operates in the `map` frame, downstream planners can treat it as a soft evidential layer that highlights likely fire sources.

---

## Wind Workflow (Simulated)

Wind sensing is currently completely simulated.  The Gazebo world does not inject wind, so the stack relies on an internal generator that can be replaced with a real sensor later.  The pipeline is implemented in three nodes:

1. **`WindSimNode` (`src/windSimNode.cpp`)**  
   - Publishes a synthetic field on `/wind/test_cloud` at a configurable period (`period_ms`).  
   - Each point contains `(x, y, z)` coordinates plus `(u, v)` velocity components (`sensor_msgs::PointCloud2` fields).  
   - The pattern mixes a uniform bias (`base_u`, `base_v`), a swirl component (`swirl_gamma`, `swirl_core2`) and smooth trigonometric noise (`noise_amp`) so the arrows in RViz feel “alive”.

2. **`WindSensNode` (`src/windSensNode.cpp`)**  
   - Subscribes to `/wind/test_cloud`, iterates each sample, and republishes it as two stamped messages:
     - `/wind/point` (`geometry_msgs::msg::PointStamped`)
     - `/wind/velocity` (`geometry_msgs::msg::Vector3Stamped`)
   - Each pair shares the same timestamp (offset by a few nanoseconds for uniqueness), emulating what a scanning probe sensor might emit while stepping through spatial samples.

3. **`WindInterpolationNode` (`src/windInterpolationNode.cpp`)**  
   - Collects the stamped points and velocities.  A hash map keyed by timestamp couples the matching samples.
   - Converts the velocity vector into an arrow marker (`visualization_msgs::msg::MarkerArray` on `/wind/markers`).  Colour and scale encode the measured speed (blue for low, red for high).
   - Marker IDs are derived from the map grid location so updates continuously refresh the same arrow instead of spawning duplicates.

> **Why simulate?** There is currently no physical wind sensor in the Gazebo loop.  The simulator acts as a drop-in replacement so you can validate the downstream processing chain in RViz or log data for algorithm development.  When a real sensor becomes available, you only need to drive `/wind/point` and `/wind/velocity` with real measurements and bypass `WindSimNode`.

---

## Node, Topic, and Parameter Reference

| Node | Executable | Key Subscriptions | Important Publications | Notable Parameters |
|------|------------|-------------------|------------------------|--------------------|
| `thermal_hotspot_node` | `thermal_hotspot_node` | `/thermal_camera/image`, `/camera/depth/image` | `/thermal_hotspot/thermal_overlay`, `/hotspots/points_stamped`, `/hotspots/points_cloud`, `/hotspots/world_points` | `hot_temp_c`, `use_percentile`, `depth_stride`, `plane_z`, `hfov_rad`, `target_frame` |
| `hotspot_mapper_node` | `hotspot_mapper_node` | `/hotspots/points_stamped`, `/hotspots/points_cloud` | `/hotspots/heatmap`, `/hotspots/heatmap_image` | `resolution`, `sigma_m`, `decay_half_life_s`, `add_weight`, `clear_threshold` |
| `wind_sim_node` | `wind_sim_node` | — | `/wind/test_cloud` | `spacing`, `extent`, `base_u`, `base_v`, `swirl_gamma`, `noise_amp`, `period_ms` |
| `wind_sensor_node` | `wind_sensor_node` | `/wind/test_cloud` | `/wind/point`, `/wind/velocity` | — |
| `wind_interpolation_node` | `wind_interpolation_node` | `/wind/point`, `/wind/velocity` | `/wind/markers` | `arrow_scale_base`, `arrow_diam`, `speed_clip_max` |

All parameters shown in the code are declared with defaults, so you can override them at launch time via YAML or CLI (`ros2 param set`).  For example, to make the hotspot detector more sensitive you can launch with `hot_temp_c:=120.0 morphology_kernel:=5`.

---

## Extending and Debugging

- **Detector tuning:** `HotspotDetector` (`src/hotspotDetector.cpp`) exposes minimum/maximum area filters, percentile-based thresholds, and morphology controls.  Tweak these parameters and re-run to balance noise rejection versus responsiveness.
- **Coordinate frames:** Thermal hotspots are projected onto a configurable plane (`plane_z`) in `target_frame` (`map` by default).  Ensure TF publishes `map -> CAMERA` and `map -> depth_sensor` transforms; otherwise the node will throttle warnings about missing transforms and skip outputs.
- **Wind visualisation:** Adjust `speed_clip_max` to match the expected range.  Values beyond the clip limit all map to the most intense colour, so set this close to your real sensor’s maximum wind speed.
- **Logging:** Each node prints throttled introspection messages (for example, the mapper logs how many cells are hot).  Use `ros2 run rclcpp_components component_container_mt` to load individual nodes if you want to isolate behaviour.

---

## Repository Structure

```
include/
  hotspotDetection_pyrosens/   # Headers for thermal detection, mapping, and transforms
  wind_pyrosens/               # Headers for simulated wind nodes
src/
  hotspotDetector.cpp          # OpenCV hotspot detection implementation
  thermalHotspotNode.cpp       # Thermal pipeline node
  hotspotMapperNode.cpp        # Heatmap aggregation node
  windSimNode.cpp              # Wind field simulator
  windSensNode.cpp             # Sensor shim for simulator output
  windInterpolationNode.cpp    # RViz marker generator
launch/
  integrated_sensors.launch.py # Launches the full stack
mains/
  main_*.cpp                   # Thin wrappers that spin each node
```

---

## Next Steps

1. Replace the simulated wind publisher with a hardware driver that publishes the same `/wind/point` + `/wind/velocity` pair.
2. Feed the heatmap into navigation or mission planning nodes to bias trajectories toward (or away from) detected thermal hotspots.
3. Capture rosbag recordings while running both Gazebo and the simulator to create repeatable scenarios for regression testing.

With the workflow above you can now walk peers or supervisors through exactly how PyroSens processes thermal data from Gazebo while keeping the wind side modular for inevitable future sensor swaps.

