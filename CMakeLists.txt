cmake_minimum_required(VERSION 2.8.3)

project(oslsim)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
  message_generation
  std_msgs
  trajectory_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  trajectory_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs # controller_manager hardware_interface roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)