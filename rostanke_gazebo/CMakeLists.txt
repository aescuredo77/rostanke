cmake_minimum_required(VERSION 3.0.2)
project(rostanke_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roseco_description
  xacro
)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})