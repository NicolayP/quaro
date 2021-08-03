cmake_minimum_required(VERSION 3.0.2)
project(quaro_control)


find_package(
  catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  controller_interface
  forward_command_controller
  pluginlib
)

catkin_package(
  CATKIN_DEPENDS controller_interface forward_command_controller
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/randomWalker.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
