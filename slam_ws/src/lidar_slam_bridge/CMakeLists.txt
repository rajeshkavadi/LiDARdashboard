cmake_minimum_required(VERSION 3.0.2)
project(lidar_slam_bridge)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs nav_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/livox_pc2_to_custom.py
  scripts/slam_web_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
