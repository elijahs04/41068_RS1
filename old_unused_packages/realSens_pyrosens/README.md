# thermal_pyrosens

Simulates a single-pixel thermal sensor and builds a blended **heat costmap** in ROS 2.

> **Note:** `thermSensNode` is not used in this setup.

---

## Quick Start (3 terminals)

### T1 — Build & RViz
```bash
# From your workspace root
colcon build --packages-select thermal_pyrosens
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

