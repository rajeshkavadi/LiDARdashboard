cmake_minimum_required(VERSION 3.0.2)
project(lidar_web_bridge)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/ros_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
