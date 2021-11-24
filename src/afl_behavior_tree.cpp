#include "behaviortree_cpp_v3/bt_factory.h"
#include <moveToReadyPoint.h>
#include <moveToGoal.h>
#include <ForkActionNode.h>
#include <raiseFork.h>
#include <moveForward.h>
#include <moveBackward.h>
#include <dropFork.h>
#include <lowerFork.h>
#include <normalDetection.h>
#include <waitForPalletDetection.h>
#include <moveActionNode.h>
#include <LowerForkActionNode.h>
#include <ForkTickDescend.h>
#include <DetectionClient.h>
#include <SleepNode.h>
#include <LoadWaypoints.h>

using namespace AFL;
using namespace BT;

int main(int argc, char** argv)
{
  BehaviorTreeFactory factory;
  ros::init(argc, argv, "afl_behavior_tree");

  factory.registerNodeType<MoveToReadyPoint>("MoveToReadyPoint");
  factory.registerNodeType<MoveActionNode>("MoveActionNode");
  factory.registerNodeType<MoveToGoal>("MoveToGoal");
  factory.registerNodeType<ForkActionNode>("ForkActionNode");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<RaiseFork>("RaiseFork");
  factory.registerNodeType<MoveBackward>("MoveBackward");
  factory.registerNodeType<DropFork>("DropFork");
  factory.registerNodeType<LowerFork>("ForkLower");
  factory.registerNodeType<WaitForPalletDetection>("WaitForPalletDetection");
  factory.registerNodeType<LowerForkActionNode>("LowerForkActionNode");
  factory.registerNodeType<ForkTickDescend>("ForkTickDescend");
  factory.registerNodeType<DetectionClient>("DetectionClient");
  factory.registerNodeType<SleepNode>("SleepNode");
  factory.registerNodeType<LoadWaypoints>("LoadWaypoints");

  std::string path;
  ros::param::get("~treeConfigPath", path);
  auto tree = factory.createTreeFromFile(path);
  tree.tickRoot();

  return 0;
}
