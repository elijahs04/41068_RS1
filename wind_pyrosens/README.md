# wind_pyrosens

ROS 2 package for simulating and visualising **wind vectors** in the PyroSENS project.  
This test package has two nodes:
- **SensorNode** – reads a test `PointCloud2` with wind data and republishes it as standard ROS messages.
- **MapperNode** – subscribes to those messages and publishes RViz arrow markers showing wind direction and speed.

---

## Package structure

```
wind_pyrosens/
├── include/wind_pyrosens/
│   ├── SensorNode.hpp
│   └── MapperNode.hpp
├── src/
│   ├── sensorNode.cpp
│   ├── mapperNode.cpp
│   ├── main_sensor.cpp
│   └── main_mapper.cpp
├── CMakeLists.txt
└── package.xml
```

---

## Nodes

### `sensor_node`
- **Subscribes**
  - `/wind/test_cloud` (`sensor_msgs/PointCloud2`)  
    Fields: `x, y, z=0, u, v, w=0` for position and velocity.
- **Publishes**
  - `/wind/point` (`geometry_msgs/PointStamped`) – wind sample position.  
  - `/wind/velocity` (`geometry_msgs/Vector3Stamped`) – wind vector at that position.

### `mapper_node`
- **Subscribes**
  - `/wind/point` (`geometry_msgs/PointStamped`)  
  - `/wind/velocity` (`geometry_msgs/Vector3Stamped`)
- **Publishes**
  - `/wind/markers` (`visualization_msgs/MarkerArray`)  
    - Each marker is an **arrow**.  
    - **Direction** = wind direction.  
    - **Length** = proportional to wind speed (capped).  
    - **Colour** = constant (white/grey) for MVP.

---

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select wind_pyrosens
source install/setup.bash
```

---

## Run

Launch the nodes in separate terminals:

```bash
ros2 run wind_pyrosens sensorNode
ros2 run wind_pyrosens mapperNode
```

Open RViz:

```bash
rviz2
```

- Add a **MarkerArray** display.  
- Topic: `/wind/markers`  
- Fixed frame: `map`

---
