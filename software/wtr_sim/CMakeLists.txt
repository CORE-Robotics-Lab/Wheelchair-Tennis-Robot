cmake_minimum_required(VERSION 3.0.2)
project(wtr_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   gazebo_ros
)

catkin_package(DEPENDS gazebo_ros)
