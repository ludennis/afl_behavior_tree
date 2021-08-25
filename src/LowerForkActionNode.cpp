#include <LowerForkActionNode.h>

namespace AFL
{

LowerForkActionNode::LowerForkActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
, mActionClient("aflSetForkHeightAction", true)
{}

BT::PortsList LowerForkActionNode::providedPorts()
{
  return {};
}

BT::NodeStatus LowerForkActionNode::tick()
{
  afl_fork_control::setForkHeightGoal forkHeightGoal;
  forkHeightGoal.targetHeight = 0.0;

  mActionClient.waitForServer();
  mActionClient.sendGoal(forkHeightGoal);

  bool success = mActionClient.waitForResult(ros::Duration(25));
  if (success)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}


} // namespace AFL
