#include "behaviortree_cpp_v3/bt_factory.h"
#include <moveToReadyPoint.h>
#include <moveToGoal.h>
#include <raiseForkActionNode.h>
#include <raiseFork.h>
#include <moveForward.h>
#include <moveBackward.h>
#include <dropFork.h>
#include <lowerFork.h>
#include <normalDetection.h>
#include <waitForPalletDetection.h>
#include <moveActionNode.h>

using namespace AFL;
using namespace BT;

int main(int argc, char** argv)
{
  BehaviorTreeFactory factory;
  ros::init(argc, argv, "afl_behavior_tree");

  factory.registerNodeType<MoveToReadyPoint>("MoveToReadyPoint");
  // factory.registerNodeType<NormalDetection>("NormalDetection");
  factory.registerNodeType<MoveActionNode>("MoveActionNode");
  factory.registerNodeType<MoveToGoal>("MoveToGoal");
  factory.registerNodeType<RaiseForkActionNode>("RaiseForkActionNode");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<RaiseFork>("RaiseFork");
  factory.registerNodeType<MoveBackward>("MoveBackward");
  factory.registerNodeType<DropFork>("DropFork");
  factory.registerNodeType<LowerFork>("ForkLower");
  factory.registerNodeType<WaitForPalletDetection>("WaitForPalletDetection");

  std::string path;
  ros::param::get("~treeConfigPath", path);
  auto tree = factory.createTreeFromFile(path);
  tree.tickRoot();

  return 0;
}
