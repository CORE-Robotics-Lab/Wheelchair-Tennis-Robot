cmake_minimum_required(VERSION 3.5)
project(ball_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wno-deprecated-declarations")

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
SET(CMAKE_BUILD_TYPE Release)
#SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

####

find_package(ZED QUIET)

if (ZED_FOUND)
  find_package(OpenCV REQUIRED)
  find_package(CUDA 11 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    dynamic_reconfigure
    cv_bridge
    image_transport
  )

  catkin_package(
    CATKIN_DEPENDS
      roscpp
  )

  include_directories(
          ${catkin_INCLUDE_DIRS}
          ${CUDA_INCLUDE_DIRS}
          ${ZED_INCLUDE_DIRS}
          ${OPENCV_INCLUDE_DIRS}
  )

  link_directories(${CUDA_LIBRARY_DIR})
  link_directories(${ZED_LIBRARY_DIR})
  link_directories(${OpenCV_LIBRARY_DIRS})
  link_directories(${catkin_LIBRARIES_DIRS})

  add_executable(ball_detection src/ball_detection.cpp)
  target_link_libraries(ball_detection ${ZED_LIBRARIES} ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})

else()
  message([WARNING] " Not Bulding Package since can't find ZED SDK!")
  find_package(catkin REQUIRED)
  catkin_package()

endif()
