# Manipulation

This repository contains source files and packages for robotic manipulation using Interbotix arms. The project includes data collection scripts and integrates various Interbotix ROS 2 packages for robotic control and visualization.

Repository Overview

Packages

data_collector (In Development)

The data_collector package provides tools for collecting robotic data, including joint states and point cloud data.

collector_script.py – Captures joint state data from the robot.

point_cloud.py – Collects and processes PointCloud2 data for 3D environment mapping.

Dependencies

This repository relies on the following ROS 2 packages:

interbotix_ros_core – Provides core firmware and communication utilities for Interbotix arms.

interbotix_ros_manipulators – Implements control and planning tools for robotic manipulators.

interbotix_ros_toolboxes – Includes various helper functions and utility scripts for robot operation.

moveit_visual_tools – Assists with visualization and debugging in MoveIt!.

Getting Started

Installation

Ensure you have ROS 2 installed and sourced. Then, clone this repository and install dependencies:

cd ~/interbotix_ws/src
git clone https://github.com/your_username/manipulation.git
cd ~/interbotix_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

Usage

To launch the data collection scripts:

ros2 run data_collector collector_script.py
ros2 run data_collector point_cloud.py

Contributing

Contributions are welcome! Feel free to open issues or submit pull requests.

License

This project is licensed under [YOUR LICENSE].

Contact

For questions or collaboration, contact [Your Name] at [your_email@example.com].

