cmake_minimum_required(VERSION 3.0.2)
project(wtr_navigation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    geometry_msgs
    tf_conversions
    teb_local_planner
    controller_manager
    control_toolbox
    eigen_conversions
    serial
)

add_message_files(
  FILES
  WheelInfo.msg
)

add_service_files(
  FILES
  TebPlans.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  teb_local_planner
)

catkin_package(
        CATKIN_DEPENDS message_runtime
        teb_local_planner
        controller_manager
        control_toolbox
        serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teb_server src/teb_server/teb_server.cpp)
target_link_libraries(teb_server
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)
add_dependencies(teb_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(teb_server ${catkin_EXPORTED_TARGETS})

add_executable(drive_hw_interface src/hardware_interface/drive_hw_interface.cpp)
target_link_libraries(drive_hw_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS drive_hw_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(serial_relay src/hardware_interface/serial_relay.cpp)
target_link_libraries(serial_relay ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS serial_relay DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
