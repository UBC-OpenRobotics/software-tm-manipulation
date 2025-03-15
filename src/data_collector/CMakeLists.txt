cmake_minimum_required(VERSION 3.5)
project(data_collector)

# Default to C++14 if no CXX standard is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interbotix_moveit_interface_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(
  MOVEIT_INTERFACE_DEPENDENCIES
    interbotix_moveit_interface_msgs
    moveit_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
)

include_directories(
  include
)

# Add the C++ executable
add_executable(commander src/data_collector/commander.cpp)
ament_target_dependencies(commander ${MOVEIT_INTERFACE_DEPENDENCIES})

# Install the C++ executable
install(TARGETS
  commander
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/data_collector/point_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/data_collector/point_extract.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/data_collector/collector_script.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/data_collector/commander.py
  DESTINATION lib/${PROJECT_NAME}
)

# Colcon build support
ament_package()
