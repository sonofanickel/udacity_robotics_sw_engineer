#!/bin/bash

xterm -e "export ROBOT_INITIAL_POSE='-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0'; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10

xterm -e roslaunch turtlebot_gazebo amcl_demo.launch &
sleep 5

xterm -e roslaunch turtlebot_rviz_launchers view_navigation.launch &
sleep 10

xterm -e roslaunch add_markers add_markers.launch &
sleep 1

xterm -e roslaunch pick_objects pick_objects.launch &

