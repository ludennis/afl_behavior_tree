#include <raiseForkActionNode.h>

namespace AFL
{

RaiseForkActionNode::RaiseForkActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
, mActionClient("aflSetForkHeightAction", true)
{}

BT::PortsList RaiseForkActionNode::providedPorts()
{
  return {
      BT::InputPort<geometry_msgs::PoseWithCovarianceStamped>("PalletPose"),
      BT::InputPort<short int>("TargetHeight"),
      BT::InputPort<short int>("PalletThickness"),
      BT::InputPort<short int>("PalletBottomPadding")
  };
}

BT::NodeStatus RaiseForkActionNode::tick()
{
  auto palletPose = getInput<geometry_msgs::PoseWithCovarianceStamped>(
      "PalletPose");
  auto palletThickness = getInput<short int>("PalletThickness");
  auto palletBottomPadding = getInput<short int>("PalletBottomPadding");
  auto targetHeight = getInput<short int>("TargetHeight");

  if (!palletPose && !targetHeight)
  {
    throw BT::RuntimeError(
        "Missing required input PalletPose and TargetHeight: ",
        palletPose.error());
    return BT::NodeStatus::FAILURE;
  }

  afl_fork_control::setForkHeightGoal forkHeightGoal;
  forkHeightGoal.targetHeight =
      (palletPose) ? abs(palletPose.value().pose.pose.position.z * 1e3 -
      palletThickness.value() / 2 + palletBottomPadding.value()) :
      targetHeight.value();

  ROS_INFO_STREAM_NAMED("[AFL|afl_behavior_tree|RaiseForkActionNode]",
      this->name() << " setting for to height " << forkHeightGoal.targetHeight
      << " mm");

  return sendForkHeightGoal(forkHeightGoal);
}

BT::NodeStatus RaiseForkActionNode::sendForkHeightGoal(
    const afl_fork_control::setForkHeightGoal &forkHeightGoal)
{
  this->mActionClient.waitForServer();
  this->mActionClient.sendGoal(forkHeightGoal);
  bool success = this->mActionClient.waitForResult(ros::Duration(25));

  if (success)
  {
    actionlib::SimpleClientGoalState state = this->mActionClient.getState();
    ROS_INFO_STREAM_NAMED("AFL", "[afl_behavior_tree]: Action finished: "
        << state.toString());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_ERROR_NAMED("AFL", "[afl_behavior_tree]: Action timed out");
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace AFL
