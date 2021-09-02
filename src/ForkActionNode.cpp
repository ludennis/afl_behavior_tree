#include <ForkActionNode.h>

namespace AFL
{

ForkActionNode::ForkActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
, mActionClient("aflSetForkHeightAction", true)
{}

BT::PortsList ForkActionNode::providedPorts()
{
  return {
      BT::InputPort<tf::StampedTransform>("PalletPose"),
      BT::InputPort<int>("TargetHeight"),
      BT::InputPort<int>("PalletThickness"),
      BT::InputPort<int>("PalletBottomPadding"),
      BT::InputPort<int>("TargetHeightOffset")
  };
}

BT::NodeStatus ForkActionNode::tick()
{
  auto palletThickness = getInput<int>("PalletThickness");
  auto palletBottomPadding = getInput<int>("PalletBottomPadding");
  auto palletPose = getInput<tf::StampedTransform>("PalletPose");
  auto targetHeight = getInput<int>("TargetHeight");
  auto targetHeightOffset = getInput<int>("TargetHeightOffset");

  if (!palletPose && !targetHeight && !targetHeightOffset)
  {
    throw BT::RuntimeError(
        "Missing required input PalletPose/TargetHeight/TargetHeightOffset: ",
        palletPose.error());
    return BT::NodeStatus::FAILURE;
  }

  afl_fork_control::setForkHeightGoal forkHeightGoal;
  if (palletPose)
  {
    forkHeightGoal.targetHeight = abs(palletPose.value().getOrigin().getZ()
        * 1e3 - palletThickness.value() / 2 + palletBottomPadding.value());
  }
  else if (targetHeight)
  {
    forkHeightGoal.targetHeight = targetHeight.value();
  }
  else
  {
    auto currentHeight = ros::topic::waitForMessage<std_msgs::UInt16>(
        "/afl/sick_DT50", ros::Duration(10));
    if (!currentHeight)
    {
      ROS_ERROR_NAMED("AFL", "[afl_behavior_tree | ForkActionNode] "
          "missing height sensor data when given TargetHeightOffset");
      return BT::NodeStatus::FAILURE;
    }
    forkHeightGoal.targetHeight = currentHeight->data + targetHeightOffset.value();
  }

  ROS_INFO_STREAM_NAMED("[AFL|afl_behavior_tree|ForkActionNode]",
      this->name() << " setting for to height " << forkHeightGoal.targetHeight
      << " mm");

  return sendForkHeightGoal(forkHeightGoal);
}

BT::NodeStatus ForkActionNode::sendForkHeightGoal(
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
