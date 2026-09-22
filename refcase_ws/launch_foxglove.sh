#!/bin/bash

# Source ROS and workspace setup
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

# Launch foxglove
roslaunch foxglove_bridge foxglove_bridge.launch