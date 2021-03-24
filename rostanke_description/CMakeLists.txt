cmake_minimum_required(VERSION 3.0.2)
project(rostanke_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
)


catkin_package()


include_directories(${catkin_INCLUDE_DIRS})
