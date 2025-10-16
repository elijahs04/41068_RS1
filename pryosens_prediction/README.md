# thermal_pyrosens

Uses Heatmap and Wind Marker Array Data and compares with threshold tables to output a predictive output on where the fire will move to **heat costmap** in ROS 2.

> **Note:** `thermSensNode` is not used in this setup.

---

## Quick Start (3 terminals)

### T1 — Build & RViz
```bash
# From your workspace root
colcon build --packages-select pyrosens_prediction
source install/setup.bash

# Open RViz with the provided config
rviz2 ~/ros2_ws/src/thermal_pyrosens/rviz/Heatmap.rviz
```

### T2 — Thermal Simulator
```bash
ros2 run thermal_pyrosens thermalSimNode
```

### T3 — Interpolator / Costmap
```bash
ros2 run thermal_pyrosens thermInterpolationNode
```

---

