#!/bin/bash

xterm -e roslaunch home_service world.launch &
sleep 5

xterm -e roslaunch home_service amcl.launch &
sleep 5

xterm -e roslaunch home_service view_navigation.launch &
sleep 5

xterm -e roslaunch add_markers add_markers.launch &