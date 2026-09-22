#!/bin/bash

CATKIN_WS=~/catkin_ws
MCAPL_DIR=~/mcapl
INTELLIJ_CMD=./idea.sh

# Terminal 1: catkin_make + Gazebo
mate-terminal --title="Gazebo Nuclear" -- bash -c "cd $CATKIN_WS && catkin_make && source devel/setup.bash && roslaunch scout_gazebo_sim gazebo_nuclear_inspection.launch; exec bash" &

# Terminal 2: LiDAR
mate-terminal --title="Open LiDAR" -- bash -c "cd $CATKIN_WS && source devel/setup.bash && roslaunch scout_gazebo_sim open_rslidar.launch; exec bash" &

# Terminal 3: IntelliJ
mate-terminal --title="IntelliJ" -- bash -c "cd $MCAPL_DIR && $INTELLIJ_CMD; exec bash" &
