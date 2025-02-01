source /opt/ros/humble/setup.bash
source install/setup.bash

echo 'export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
