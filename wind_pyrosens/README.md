# pyrosens_heat

A ROS 2 package for simulating a heat sensor and visualising heat distribution as a 2-D heatmap (`nav_msgs/OccupancyGrid`).

It contains two main nodes:

- **sensorNode**  
  Publishes synthetic temperature samples (`sensor_msgs/Temperature`) and their positions (`geometry_msgs/PointStamped`).  
  Can also replay a batch of points from a `sensor_msgs/PointCloud` topic for testing.

- **mapperNode**  
  Subscribes to the sensor data and produces a 2-D heatmap. Each sample “paints” a circular Gaussian brush into the grid.  
  Heatmap is published on `/heat/heatmap`.

---

## Build instructions

From the root of your ROS 2 workspace:

```bash
# ---- Terminal 1 ----
# Step 1: move into the workspace
cd ~/ros2_ws

# Step 2: build only the pyrosens_heat package
colcon build --packages-select pyrosens_heat

# Step 3: source the build so ROS can find it
source install/setup.bash

# Step 4: publish a static transform from world -> map
# (this ensures the "map" frame exists for RViz)
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id world --child-frame-id map

# ---- Terminal 2 ----
# Run the heatmap generator
ros2 run pyrosens_heat mapperNode

# ---- Terminal 3 ----
# Run the synthetic heat sensor
ros2 run pyrosens_heat sensorNode

# ---- Terminal 4 ----
# Publish a one-shot test cloud with 5 points and temperatures
ros2 topic pub -1 /heat/test_cloud sensor_msgs/PointCloud "{
  header: {frame_id: map},
  points: [
    {x: -5.0, y: 0.0, z: 0.0},
    {x: -2.0, y: 1.0, z: 0.0},
    {x:  0.0, y: 0.0, z: 0.0},
    {x:  2.0, y: 0.0, z: 0.0},
    {x:  5.0, y: -1.0, z: 0.0}
  ],
  channels: [{name: 'temperature',
              values: [30.0, 35.0, 45.0, 75.0, 40.0]}]
}"

# ---- Terminal 5 ----
# Launch RViz with a pre-saved configuration
rviz2 -d ~/ros2_ws/src/pyrosens_heat/rviz/heatmap_test.rviz

