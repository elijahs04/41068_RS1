# wind_pyrosens

Wind sensing + visualization package for PyroSENS.  
This package simulates wind vectors over a 2D grid and displays them in RViz as arrows whose **length, thickness, and color** vary with velocity.

---

## Build

T1:
colcon build --packages-select wind_pyrosens
source install/setup.bash

---

## Run

Open 6 terminals and run the following commands:

T1: Build and source
colcon build --packages-select wind_pyrosens
source install/setup.bash

T2: RViz (with config for wind markers)
rviz2 -d ~/ros2_ws/src/wind_pyrosens/rviz/WindMarkerArray.rviz

T3: Static transform (world → map)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map

T4: Mapper node (subscribes to wind points + velocities, publishes markers)
ros2 run wind_pyrosens mapperNode

T5: Sensor node (publishes wind points + velocities from a test cloud)
ros2 run wind_pyrosens sensorNode

T6: Test cloud publisher (generates a 4×4 m grid of points with simulated wind)
ros2 run wind_pyrosens testCloudPub

---

## What You’ll See

- **RViz** will display a 4 m × 4 m grid of arrows at 0.5 m spacing.
- Arrow **length and thickness** scale with wind speed.
- Arrow **color** transitions from **blue (slow)** → **red (fast)**.
- Wind field combines a dominant background flow, a swirl around the origin, and small random variations.

---

## Parameters

You can tune the test cloud publisher at runtime:

ros2 param set /test_cloud_pub base_u 0.8
ros2 param set /test_cloud_pub base_v 1.2
ros2 param set /test_cloud_pub swirl_gamma 0.8
ros2 param set /test_cloud_pub period_ms 200

---

## Notes

- The static transform (T3) is required so RViz recognizes `map` in the TF tree.
- Arrows update in place each tick (stable IDs per grid cell).
- This package is meant as a **test harness**; when integrated into the full PyroSENS system, the `SensorNode` will subscribe to actual sensor data instead of the synthetic test cloud.
