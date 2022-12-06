cmake_minimum_required(VERSION 3.0.2)
project(wtr_plan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    wtr_navigation
    wam_control
    wtr_sim
)

catkin_package(
      CATKIN_DEPENDS
        wtr_navigation
        wam_control
        wtr_sim
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
