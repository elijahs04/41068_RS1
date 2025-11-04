pyrosens_hotspotDetection

Hotspot detection from thermal images in Ignition Gazebo, with ROS 2 (Humble).
Step 1 focuses on: subscribe to thermal image → find hottest pixel → log (u, v) and °C.
Later steps can extend to blob detection, tracking, overlays, and 2D thermal point sets.

Prerequisites

Ubuntu 22.04 + ROS 2 Humble

Ignition/Gazebo (Fortress/Garden/Harmonic – whichever you use with your sim)

Packages:

rclcpp, sensor_msgs, cv_bridge, vision_msgs

OpenCV (installed via system + ROS cv_bridge)

A Gazebo–ROS bridge mapping your thermal topic to ROS:

- ros_topic_name: "/thermal_camera/image"
  gz_topic_name:  "/model/parrot/thermal_camera/image"   # adjust to your world
  ros_type_name:  "sensor_msgs/msg/Image"
  gz_type_name:   "gz.msgs.Image"                        # confirm with `gz topic -i`
  direction:      GZ_TO_ROS


✅ Confirm the thermal image encoding is mono8 or mono16:

ros2 topic echo /thermal_camera/image --once


Look for encoding: mono8 or mono16 and correct width/height.

Build

From the workspace root (e.g. ~/41068_ws or your project’s git root that contains this package):

# (optional) clean
rm -rf build install log

colcon build --packages-select pyrosens_hotspotDetection
source install/setup.bash


⚠️ Naming warning: ROS suggests lowercase-with-underscores.
This package is named pyrosens_hotspotDetection. It’s fine to use as-is, but you can rename to pyrosens_hotspot_detection later if you want to silence warnings.

Run the simulation & RViz

Start your sim stack (as you already do):

ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py \
  slam:=true nav2:=true rviz:=true world:=test_world


Ensure the thermal bridge is active and /thermal_camera/image is publishing.

Run the hotspot detector (Step 1)
# Terminal with the same workspace sourced
ros2 run pyrosens_hotspotDetection thermalHotspotNode \
  --ros-args \
  -p thermal_topic:=/thermal_camera/image \
  -p frame_id:=thermal_camera_frame \
  -p temp_gain:=0.3921568627 \
  -p temp_offset:=0.0


temp_gain and temp_offset linearly map raw pixel → °C:
temp_c = temp_gain * raw + temp_offset
Example above maps 0–255 → 0–100 °C (adjust for your sim).

You should see periodic logs like:

[INFO] Hotspot: (u=..., v=...) raw=... temp=...°C (img WxH, mono8/mono16)

Parameters
Name	Type	Default	Description
thermal_topic	string	/thermal_camera/image	ROS topic for the thermal image.
frame_id	string	thermal_camera_frame	Frame stamped in any outputs (used later).
temp_gain	double	1.0	Linear map gain from raw → °C.
temp_offset	double	0.0	Linear map offset from raw → °C.

If you don’t know the mapping yet, set temp_gain=1.0, temp_offset=0.0 and treat logs as raw counts.

Topics

Input

/thermal_camera/image (sensor_msgs/Image, mono8 or mono16)

Output (Step 1)

No published topics yet; the node logs the hottest pixel location and °C.

(Next steps will add /thermal_hotspots, /thermal_overlay, /thermal_points2d.)

Troubleshooting

I still see RGB, not thermal
You’re likely bridging the RGB camera. Fix the bridge to point to the thermal sensor’s Gazebo topic. Confirm with:

gz topic -l
gz topic -i /world/<world>/model/parrot/thermal_camera/image


Encoding is not mono8/mono16
Update your sensor SDF:
<image><format>L8</format></image> or <format>L16</format>, and adjust your bridge.

No messages on /thermal_camera/image
Check the bridge is running and namespaces match your world & model names.

Build errors about missing files
Verify source names in CMakeLists.txt match your actual files:

src/thermal_hotspot_node.cpp

src/hotspot_detector.cpp

src/main.cpp
And headers under include/thermal_hotspot_detector/.