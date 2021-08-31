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
      BT::InputPort<tf::StampedTransform>("PalletPose"),
      BT::InputPort<short int>("TargetHeight"),
      BT::InputPort<short int>("PalletThickness"),
      BT::InputPort<short int>("PalletBottomPadding"),
      BT::InputPort<short int>("TargetHeightOffset")
  };
}

BT::NodeStatus RaiseForkActionNode::tick()
{
  auto palletThickness = getInput<short int>("PalletThickness");
  auto palletBottomPadding = getInput<short int>("PalletBottomPadding");
  auto palletPose = getInput<tf::StampedTransform>("PalletPose");
  auto targetHeight = getInput<short int>("TargetHeight");
  auto targetHeightOffset = getInput<short int>("TargetHeightOffset");

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
      ROS_ERROR_NAMED("AFL", "[afl_behavior_tree | RaiseForkActionNode] "
          "missing height sensor data when given TargetHeightOffset");
      return BT::NodeStatus::FAILURE;
    }
    forkHeightGoal.targetHeight = currentHeight->data + targetHeightOffset.value();
  }

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
