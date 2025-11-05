# Pyrosens Integrated Sensors

# Terminal 1
Heatspot detection needs sim running to work

```bash
cd ~/ros2_ws
colcon build --packages-select 41068_ignition_bringup
source ~/ros2_ws/install/setup.bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=test_world
```

# Terminal 2
```bash
cd ~/ros2_ws
colcon build --packages-select pyrosens_integrated_sensors
source ~/ros2_ws/install/setup.bash
ros2 launch pyrosens_integrated_sensors integrated_sensors.launch.py
```