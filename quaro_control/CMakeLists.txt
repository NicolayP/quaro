cmake_minimum_required(VERSION 3.0.2)
project(quaro_control)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package()
