cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)

# rospy was added as a dependancy
find_package(catkin REQUIRED COMPONENTS
rospy
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
