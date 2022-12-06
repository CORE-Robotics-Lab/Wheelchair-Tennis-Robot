cmake_minimum_required(VERSION 3.0.2)
project(ball_localization)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  eigen_conversions
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  tf2_ros)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)
if(NOT YAML_CPP_FOUND)
  find_package(yaml-cpp REQUIRED)
endif()


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

set(ROSLINT_CPP_OPTS "--ball_filter=-build/c++17,-runtime/references")
roslint_cpp()

###################################
## catkin specific configuration ##
###################################
add_service_files(
  FILES
    GetState.srv
    SetDatum.srv
    SetPose.srv
    SetUTMZone.srv
    ToggleFilterProcessing.srv
    FromLL.srv
    ToLL.srv
)

generate_messages(
  DEPENDENCIES
    geographic_msgs
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ball_ekf
    ball_ekf_localization_nodelet
    ball_filter_base
    ball_filter_utilities
    ball_navsat_transform
    ball_navsat_transform_nodelet
    ros_ball_filter
    ros_ball_filter_utilities
    ball_localization_estimator
    ros_ball_localization_listener
    ball_ukf
    ball_ukf_localization_nodelet
  CATKIN_DEPENDS
    cmake_modules
    diagnostic_msgs
    diagnostic_updater
    eigen_conversions
    geographic_msgs
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    ${EIGEN_PACKAGE}
    YAML_CPP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

# Library definitions
add_library(ball_filter_utilities src/ball_filter_utilities.cpp)
add_library(ball_filter_base src/ball_filter_base.cpp)
add_library(ball_ekf src/ball_ekf.cpp)
add_library(ball_ukf src/ball_ukf.cpp)
add_library(ball_localization_estimator src/ball_localization_estimator.cpp)
add_library(ros_ball_localization_listener src/ros_ball_localization_listener.cpp)
add_library(ros_ball_filter_utilities src/ros_ball_filter_utilities.cpp)
add_library(ros_ball_filter src/ros_ball_filter.cpp)
add_library(ball_navsat_transform src/ball_navsat_transform.cpp)
add_library(ball_ekf_localization_nodelet src/ball_ekf_localization_nodelet.cpp)
add_library(ball_ukf_localization_nodelet src/ball_ukf_localization_nodelet.cpp)
add_library(ball_navsat_transform_nodelet src/ball_navsat_transform_nodelet.cpp)

# Executables
add_executable(ball_ekf_localization_node src/ball_ekf_localization_node.cpp)
add_executable(ball_ukf_localization_node src/ball_ukf_localization_node.cpp)
add_executable(ball_navsat_transform_node src/ball_navsat_transform_node.cpp)
add_executable(ball_localization_listener_node src/ball_localization_listener_node.cpp)

# Dependencies
add_dependencies(ball_filter_base ${PROJECT_NAME}_gencpp)
add_dependencies(ball_navsat_transform ${PROJECT_NAME}_gencpp)
add_dependencies(ball_localization_listener_node ${PROJECT_NAME}_gencpp)

# Linking
target_link_libraries(ros_ball_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ball_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ball_filter_base ball_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ball_ekf ball_filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ball_ukf ball_filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ros_ball_filter ball_ekf ball_ukf ros_ball_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ball_localization_estimator ball_filter_utilities ball_filter_base ball_ekf ball_ukf ${EIGEN3_LIBRARIES})
target_link_libraries(ros_ball_localization_listener ball_localization_estimator ros_ball_filter_utilities
  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(ball_localization_listener_node ros_ball_localization_listener ${catkin_LIBRARIES})
target_link_libraries(ball_ekf_localization_node ros_ball_filter ${catkin_LIBRARIES})
target_link_libraries(ball_ekf_localization_nodelet ros_ball_filter ${catkin_LIBRARIES})
target_link_libraries(ball_ukf_localization_node ros_ball_filter ${catkin_LIBRARIES})
target_link_libraries(ball_ukf_localization_nodelet ros_ball_filter ${catkin_LIBRARIES})
target_link_libraries(ball_navsat_transform ball_filter_utilities ros_ball_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${GeographicLib_LIBRARIES})
target_link_libraries(ball_navsat_transform_node ball_navsat_transform ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
target_link_libraries(ball_navsat_transform_nodelet ball_navsat_transform ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ball_ekf
  ball_ekf_localization_nodelet
  ball_filter_base
  ball_filter_utilities
  ball_navsat_transform
  ball_navsat_transform_nodelet
  ros_ball_filter
  ros_ball_filter_utilities
  ball_localization_estimator
  ros_ball_localization_listener
  ball_ukf
  ball_ukf_localization_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS
  ball_ekf_localization_node
  ball_navsat_transform_node
  ball_localization_listener_node
  ball_ukf_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
