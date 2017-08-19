cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control)

find_package(catkin REQUIRED COMPONENTS
  # TODO: add dependencies
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
