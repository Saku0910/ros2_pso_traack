# Opencv_PSO_track_using_ROS2

This is the ROS 2 packages for object tracking using OpenCV and PSO.
This repository is set up to track targets by hue and saturation.
Other targets can be tracked by changing them.


## Requirement
- Laptop PC
  - Ubuntu 20.04 Foxy
- Realsense D455

## Build
```shell: Terminal
sudo nano ~/.bashrc
cd ~/dev_ws
colcon build --packages-select ros2_pso_track
```

## Usage
```shell: Terminal
cd ~/dev_ws
. install/setup.bash
ros2 run ros2_pso_track rs_track
```
Open shells. Run the ros2_intel_realsense node
```shell: Terminal
cd ~/ros2_ws
. install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```
