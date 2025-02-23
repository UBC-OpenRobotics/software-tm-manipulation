# Manipulation

This repository contains source files and packages for robotic manipulation using Interbotix arms. The project includes data collection scripts and integrates various Interbotix ROS 2 packages for robotic control and visualization.

## Important Links
- Draw.io: https://drive.google.com/file/d/1GFC-9pm8g2PJJFznMbJC_jBeHdeerQDY/view?usp=sharing


## Repository Overview

### Packages
#### `data_collector` (In Development)
The `data_collector` package provides tools for collecting robotic data, including joint states and point cloud data.

- **`collector_script.py`** – Captures joint state data from the robot.
- **`point_cloud.py`** – Collects and processes PointCloud2 data for 3D environment mapping.

### Dependencies
This repository relies on the following ROS 2 packages:

- **[`interbotix_ros_core`](https://github.com/Interbotix/interbotix_ros_core)** – Provides core firmware and communication utilities for Interbotix arms.
- **[`interbotix_ros_manipulators`](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages.html)** – Implements control and planning tools for robotic manipulators.
- **[`interbotix_ros_toolboxes`](https://github.com/Interbotix/interbotix_ros_toolboxes)** – Includes various helper functions and utility scripts for robot operation.
- **[`moveit_visual_tools`](https://github.com/ros-planning/moveit_visual_tools)** – Assists with visualization and debugging in MoveIt!.

## Getting Started

### Installation
Ensure you have ROS 2 installed and sourced. Then, clone this repository and install dependencies:

```bash
cd ~/interbotix_ws/src
git clone https://github.com/your_username/manipulation.git
cd ~/interbotix_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Usage
To launch the data collection scripts:

```bash
ros2 run data_collector collector_script.py
ros2 run data_collector point_cloud.py
```

### Training Environment

```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx150 hardware_type:=gz_classic
```

### Rviz2 Visualization

To render pointCloud2 data one should change the global frame to `camera_depth_frame`





### Realsense Camera integration

```bash
# git clone
cd ~/manipulation/src/
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# init
cd ~/manipulation
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -r -y
# OPTIONAL : downgrade for cv
pip install "numpy<2"

# build
colcon build

# Source
ROS_DISTRO=humble
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/manipulation
. install/local_setup.bash

# test
ros2 run realsense2_camera realsense2_camera_node
# Success : expecting of <RealSense Node Is Up!>
```


#### Camera Node interation

Function calls (bold == default value): 
* image_types : `**rgb_frame**` 
* frame_rate  : `6`, `15`, `30`, `**60**`
* mirror  : `true`, `**false**`

```bash
# Publisher
ros2 run intel_d435_camera camera_pub --ros-args --param image_type:=rgb_frame --param frame_rate:=30
# Subscriber
ros2 run intel_d435_camera camera_sub --ros-args --param image_type:=rgb_frame --param mirror:=true
```

Make new Files
```bash
cd ~/manipulation/src/intel_d435_camera/intel_d435_camera
touch [fileName].py
# local build/source
colcon build --packages-select intel_d435_camera --symlink-install
source install/setup.bash
```