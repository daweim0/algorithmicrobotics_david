cmake_minimum_required(VERSION 2.8.3)
project(dagu_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  adafruit_drivers
  duckietown_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
