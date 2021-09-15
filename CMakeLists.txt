cmake_minimum_required(VERSION 3.0.2)
project(afl_behavior_tree)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  DEPENDS aflctrl2 afl_fork_control
)
find_package(BehaviorTreeV3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
  afl_detection
  tf
  aflctrl2
  afl_fork_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/afl_behavior_tree.cpp
  src/shared_class.cpp
  src/fork_insertion_controller.cpp
  src/moveToReadyPoint.cpp
  src/moveToGoal.cpp
  src/ForkActionNode.cpp
  src/raiseFork.cpp
  src/moveForward.cpp
  src/moveBackward.cpp
  src/dropFork.cpp
  src/lowerFork.cpp
  src/waitForPalletDetection.cpp
  src/moveActionNode.cpp
  src/LowerForkActionNode.cpp
  src/ForkTickDescend.cpp
)

target_link_libraries(${PROJECT_NAME}
  BT::behaviortree_cpp_v3
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
