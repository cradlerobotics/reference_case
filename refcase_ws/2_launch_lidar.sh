
#!/bin/bash

# Source ROS and workspace setup

source /opt/ros/melodic/setup.bash

source /catkin_ws/devel/setup.bash

# roslaunch velodyne_pointcloud VLP16_points.launch

roslaunch scout_bringup launch_lidar.launch