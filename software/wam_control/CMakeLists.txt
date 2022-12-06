cmake_minimum_required(VERSION 3.0.2)
project(wam_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_visual_tools
    tf_conversions
    controller_manager
    control_toolbox
    eigen_conversions
    wam_moveit
    wam_model
    ball_calibration
    trajectory_msgs
)

add_service_files(
  FILES
  LinearSwing.srv
  Swing.srv
  TrajectoryProfiler.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
        moveit_core
        moveit_ros_planning_interface
        moveit_ros_move_group
        moveit_visual_tools
        controller_manager
        control_toolbox
        wam_moveit
        wam_model
        ball_calibration
        trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cartesian_path src/swings/cartesian_path.cpp)
target_link_libraries(cartesian_path ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cartesian_path DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(linear_swing src/swings/linear_swing.cpp)
target_link_libraries(linear_swing ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS linear_swing DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
add_dependencies(linear_swing ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(linear_swing ${catkin_EXPORTED_TARGETS})

add_executable(arm_hw_interface_sim src/hardware_interface/arm_hw_interface_sim.cpp)
target_link_libraries(arm_hw_interface_sim ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS arm_hw_interface_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trajectory_profiler src/trajectory_profiler/trajectory_profiler.cpp)
target_link_libraries(trajectory_profiler ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS trajectory_profiler DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
add_dependencies(trajectory_profiler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(swing_server src/swing/swing_server.cpp)
target_link_libraries(swing_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS swing_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
add_dependencies(swing_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


