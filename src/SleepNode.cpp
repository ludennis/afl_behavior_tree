#include <SleepNode.h>

namespace AFL
{

SleepNode::SleepNode(
    const std::string &name, const BT::NodeConfiguration &config)
: SyncActionNode(name, config)
{}

BT::PortsList SleepNode::providedPorts()
{
  return {
      BT::InputPort<double>("SleepTimeInSeconds")
  };
}

BT::NodeStatus SleepNode::tick()
{
  auto sleepTime = getInput<double>("SleepTimeInSeconds");

  ros::Duration(sleepTime.value_or(1.0)).sleep();

  return BT::NodeStatus::SUCCESS;
}

} // namespace AFL
