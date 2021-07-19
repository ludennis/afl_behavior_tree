cmake_minimum_required(VERSION 3.0.2)
project(afl_behavior_tree)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(BehaviorTreeV3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  afl_fork_control
  detection
  aflctrl_msgs
  tf2
  tf2_ros
)

catkin_package()

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/afl_behavior_tree.cpp
  src/afl_behavior.cpp
  src/shared_class.cpp
  src/fork_insertion_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  BT::behaviortree_cpp_v3
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
