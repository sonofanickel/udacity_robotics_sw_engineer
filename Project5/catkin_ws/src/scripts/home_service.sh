#!/bin/bash

xterm -e roslaunch home_service world.launch &
sleep 5

xterm -e roslaunch home_service amcl.launch &
sleep 5

xterm -e roslaunch home_service view_navigation.launch &
sleep 5

xterm -e roslaunch add_markers add_markers.launch &
sleep 1

xterm -e roslaunch pick_objects pick_objects.launch &
