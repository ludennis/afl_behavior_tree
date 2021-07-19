#include "behaviortree_cpp_v3/bt_factory.h"
#include "afl_behavior.h"

using namespace AFL;
using namespace BT;

int main(int argc, char** argv)
{
  BehaviorTreeFactory factory;
  ros::init(argc, argv, "afl_behavior_tree");

  factory.registerNodeType<Move2RP>("Move2RP");
  // factory.registerNodeType<NormalDetection>("NormalDetection");
  factory.registerNodeType<Move2Goal>("Move2Goal");
  factory.registerNodeType<Move2Height>("Move2Height");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<ForkHigher>("ForkHigher");    
  factory.registerNodeType<MoveBackward>("MoveBackward");
  factory.registerNodeType<ForkDown>("ForkDown");
  factory.registerNodeType<ForkLower>("ForkLower");

  std::string path;
  ros::param::get("~treeConfigPath", path);
  auto tree = factory.createTreeFromFile(path);
  tree.tickRoot();

  return 0;
}
