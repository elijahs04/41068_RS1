# pyrosens_progressC3

Unified ROS 2 (Humble) package for **thermal** and **wind** perception:

- Thermal & wind **simulators** (publish synthetic data)
- Thermal **interpolation** → `nav_msgs/OccupancyGrid` heatmap + sample cloud
- Wind **interpolation / visualization** → `visualization_msgs/MarkerArray`
- **Goals / movement** helper (Nav2 action client)
- Multiple **runners (mains)** to spin subsystems together or individually

> Target: Ubuntu 22.04 • ROS 2 Humble

---

## Repository layout

    pyrosens_progressC3/
    ├─ cmake/                         # CMake fragments (included by root CMakeLists)
    │  ├─ integration.cmake
    │  ├─ movement.cmake
    │  ├─ thermal.cmake
    │  └─ wind.cmake
    ├─ include/
    │  ├─ goals_pyrosens/
    │  │  └─ goals.hpp
    │  ├─ thermal_pyrosens/
    │  │  ├─ thermalSimNode.hpp
    │  │  ├─ thermInterpolationNode.hpp
    │  │  └─ thermSensNode.hpp
    │  └─ wind_pyrosens/
    │     ├─ windInterpolationNode.hpp
    │     ├─ windSensNode.hpp
    │     └─ windSimNode.hpp
    ├─ mains/                         # entry points (each contains a main())
    │  ├─ main_perception.cpp         # thermal + wind sensors + interpolations (all four)
    │  ├─ main_interpolation.cpp      # thermal + wind interpolation only
    │  ├─ main_thermal.cpp            # thermal sensor + interpolation
    │  ├─ main_wind.cpp               # wind sensor + interpolation
    │  ├─ main_thermalSim.cpp         # thermal simulator
    │  └─ main_windSim.cpp            # wind simulator
    ├─ rviz/
    │  ├─ Heatmap.rviz
    │  └─ WindMarkerArray.rviz
    ├─ src/                           # node implementations (no mains here)
    │  ├─ goals.cpp
    │  ├─ thermalSimNode.cpp
    │  ├─ thermInterpolationNode.cpp
    │  ├─ thermSensNode.cpp
    │  ├─ windInterpolationNode.cpp
    │  ├─ windSensNode.cpp
    │  └─ windSimNode.cpp
    ├─ CMakeLists.txt                 # root (the only one that calls ament_package)
    └─ package.xml

---

## Build

    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select pyrosens_progressC3
    source install/setup.bash

    colcon build --packages-select pyrosens_progressC3 --cmake-clean-cache
    source install/setup.bash

---

## Runners / binaries

| Binary                 | Spins (nodes)                                                                  | Typical use                                 |
|------------------------|--------------------------------------------------------------------------------|---------------------------------------------|
| `main_perception`      | Thermal **Sensor + Interpolation**; Wind **Sensor + Interpolation**            | Full perception from sensors/sims           |
| `main_interpolation`   | Thermal **Interpolation** + Wind **Interpolation** only                        | If sensors are external                     |
| `main_thermal`         | Thermal **Sensor + Interpolation**                                             | Debug thermal stack                         |
| `main_wind`            | Wind **Sensor + Interpolation**                                                | Debug wind stack                            |
| `main_thermalSim`      | Thermal **Simulator** (Temperature + PointStamped)                             | Drive thermal pipeline without hardware     |
| `main_windSim`         | Wind **Simulator** (PointCloud2 with fields `x,y,z,u,v`)                       | Drive wind pipeline without hardware        |

> Optional single-node mains (only if present in your repo): `main_thermSens`, `main_thermInterpolation`, `main_windSens`, `main_windInterpolation`.

---

## How to run

### A) Full sim + perception + RViz (current setup, 4 terminals)

Terminal 1 – simulator

    ros2 run pyrosens_progressC3 main_sim


Terminal 3 – Perception (spins thermal+wind sensor+interpolation)

    ros2 run pyrosens_progressC3 main_perception

Terminal 4 – RViz (pick a config)

    rviz2 -d $(ros2 pkg prefix pyrosens_progressC3)/share/pyrosens_progressC3/rviz/Heatmap.rviz
    # or
    rviz2 -d $(ros2 pkg prefix pyrosens_progressC3)/share/pyrosens_progressC3/rviz/WindMarkerArray.rviz

### B) Thermal-only or Wind-only bring-up (2–3 terminals)

Thermal stack

    ros2 run pyrosens_progressC3 main_thermalSim
    ros2 run pyrosens_progressC3 main_thermal

Wind stack

    ros2 run pyrosens_progressC3 main_windSim
    ros2 run pyrosens_progressC3 main_wind

### C) Interpolation only (sensors are external)

    ros2 run pyrosens_progressC3 main_interpolation

> Later, you can add a combined `main_sim` to reduce to 3 terminals: `main_sim` (both sims) + `main_perception` + RViz.

---

## Topics (defaults)

**ThermalSimNode**
- Publishes: `/heat/sample_point` (`geometry_msgs/PointStamped`)
- Publishes: `/heat/temperature`  (`sensor_msgs/Temperature`)

**ThermInterpolationNode**
- Subscribes: `/heat/sample_point`, `/heat/temperature`
- Publishes: `/heat/costmap` (`nav_msgs/OccupancyGrid`)
- Publishes: `/heat/samples` (`sensor_msgs/PointCloud2`)

**WindSimNode**
- Publishes: `/wind/test_cloud` (`sensor_msgs/PointCloud2` with fields `x,y,z,u,v`)

**WindInterpolationNode**
- Subscribes: `/wind/test_cloud`
- Publishes: `/wind/markers` (`visualization_msgs/MarkerArray`)

---

## Parameters (high-value)

**ThermalSimNode**
- `frame_id` (string, `"map"`)
- `period_ms` (int, 60)
- `T_ambient` (double, 25.0), `T_peak` (double, 550.0), `noise_std` (double, 1.5)
- Starburst scan: `center_x`, `center_y`, `dr`, `dtheta_deg`, `omega_rps`, `scan_r_max`, `samples_per_tick`
- Blob shape: `blob_n` (2.6), `blob_beta` (8.0)

**WindSimNode**
- `spacing` (double, 0.5), `extent` (double, 2.0)
- Base flow: `base_u`, `base_v`
- Swirl: `swirl_gamma`, `swirl_core2`
- `noise_amp` (double, 0.2)
- `period_ms` (int, 500)

Examples (at runtime):

    ros2 param set /thermal_sim_node T_peak 650.0
    ros2 param set /wind_sim_node swirl_gamma 0.6

---

## Troubleshooting

- **Unknown package in `--packages-select`**  
  Build name must match `<name>` in `package.xml`. Check with `colcon list`.

- **`package.xml` name ≠ `project(...)` name**  
  Make them identical; clean build:  
      rm -rf build/ install/ log/  
      colcon build --packages-select pyrosens_progressC3 --cmake-clean-cache

- **`undefined reference to 'main'`**  
  You are linking sources without a `main()` as an executable. Build nodes as **libraries** and create executables from files in `mains/`.

- **Header not found**  
  Don’t prefix includes with `include/`. Use installed include paths, e.g.:  
      #include "thermal_pyrosens/thermalSimNode.hpp"

- **Stale environment warnings (AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH)**  
  Open a fresh terminal and `source /opt/ros/humble/setup.bash` then `source install/setup.bash`.

---

## License & maintainers

- License: **LGPLv3** (see `package.xml`)
- Maintainers: update `<maintainer>` entries in `package.xml` (multiple allowed)
- Contributors: add `<author>` entries as desired
- README generated by Chat GPT
