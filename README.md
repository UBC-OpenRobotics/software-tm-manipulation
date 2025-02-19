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
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

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
