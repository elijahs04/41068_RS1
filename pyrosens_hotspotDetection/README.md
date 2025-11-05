# pyrosens_hotspotDetection

**Thermal Hotspot Detection and Mapping in ROS 2 (Humble)**

This package provides a complete pipeline for detecting thermal hotspots from a simulated or real thermal camera and generating a persistent, decaying 2D heatmap of fire or high-temperature regions in the environment.

It is part of the **PyroSens** project for UAV-based fire surveying and hotspot prediction.

---

## Overview

The system consists of **two ROS 2 nodes**:

| Node | Purpose |
|------|----------|
| **`thermalHotspotNode`** | Subscribes to thermal and depth images, detects hotspot pixels using OpenCV, projects them into 3D world coordinates, and publishes stamped points and a point cloud. |
| **`hotspotMapperNode`** | Subscribes to the published hotspot points, accumulates them into a decaying 2D heatmap, and publishes both an OccupancyGrid (`/hotspots/heatmap`) and a colorized image (`/hotspots/heatmap_image`) for visualization. |

Together, they form a modular perception pipeline for detecting and visualizing thermal activity in real time.

---

## Features

- Thermal hotspot detection using adaptive thresholding and morphological filtering  
- Projection from camera frame → world frame using TF transforms  
- Point cloud & pose publishing of detected hotspots  
- Dynamic 2D heatmap accumulation with Gaussian splats and temporal decay  
- Modular architecture (detector and mapper can run separately or together)  
- Live visualization in RViz2 (point clouds, heatmap image, occupancy grid)

---

## Dependencies

This package uses standard ROS 2 Humble libraries:

- rclcpp  
- sensor_msgs  
- geometry_msgs  
- nav_msgs  
- std_srvs  
- cv_bridge  
- vision_msgs  
- tf2, tf2_ros, tf2_geometry_msgs  
- OpenCV  
- Eigen3  
- message_filters  

All dependencies are declared in the `package.xml`.

---

## Building

Clone the package into your ROS 2 workspace (for example `ros2_ws/src`) and build:

```bash
cd ~/ros2_ws
colcon build --packages-select pyrosens_hotspotDetection
source install/setup.bash
```

Running the Nodes
Option 1 — Launch both nodes and RViz

```bash
ros2 launch pyrosens_hotspotDetection hotspot_detection.launch.py
```
This will:
1. Start the thermalHotspotNode (detects and publishes hotspots)
2. Start the hotspotMapperNode (builds and publishes the heatmap)
3. Launch RViz2 with the preset configuration rviz/hotspot_detection.rviz