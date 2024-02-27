cmake_minimum_required(VERSION 2.8.3)
project(home_service)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY map
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY world
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
