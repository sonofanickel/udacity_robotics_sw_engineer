#!/bin/bash

xterm -e roslaunch home_service world.launch &
sleep 5

xterm -e roslaunch home_service slam.launch &
sleep 5

xterm -e roslaunch home_service view_navigation.launch &
sleep 5

xterm -e roslaunch home_service keyboard_teleop.launch &
