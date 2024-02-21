# Project 5: Home Service Robot

## Purpose

The purpose of this project is to build a mobile robot that utilizing localization, mapping, object avoidance and path planning to "carry" a simulated 3D object from one location to another.

## Packages

The following ROS packages are utilized by this project:

* [gmapping](http://wiki.ros.org/gmapping)
The gmapping package provides a Simultaneous Localization and Mapping (SLAM) function as a ROS node called slam_gmapping. We use slam_gmapping to create a 2-D occupancy grid map of our simulated world. Slam_gmapping uses data from a laser rangefinder and pose collected by the mobile robot to generate a map and localize the robot within the map, both of which are visible in RViz. A screenshot of the generated map for our world can be seen in the map/ directory.
* [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
Allows the user to control the mobile robot, directing it around the simulated world. This project uses the keyboard for control.
* [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
Provides ROS launch files and rviz configurations for viewing different kinds of information within RViz using Turtlebot. For this project we use a customized version of the "navigation" RViz configuration that additionally displays virtual Marker objects - which can be simple 3D shapes - which will be "carried" from one location to another by the robot.
* [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)
Provides convenience ROS launch files, gazebo world files, and a 2D occupancy map that are used in conjunction with the other included ROS packages to demonstrate the project goals.

Additionally, the following custom packages were developed:
* pick_objects
Instantiates a ROS node that interacts with the ROS Navigation stack. It sends messages to the navigation stack that instruct the robot to move to the desired location. Once the location is reached (if it is reached), it send a message to the add_markers node that informs it which location it has reached (pickup or drop-off)  and what the X-Y coordinates of the location is.
* add_markers
Instantiates a ROS node that creates and deletes Marker objects within the simulated environment. Subscribes to messages published by pick_objects that inform it if a Marker object has been picked up or dropped off, and the X-Y location of each event.

