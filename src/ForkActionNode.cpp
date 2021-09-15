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
      BT::InputPort<double>("TargetHeight"),
      BT::InputPort<double>("PalletThickness"),
      BT::InputPort<double>("PalletBottomPadding"),
      BT::InputPort<double>("TargetHeightOffset"),
      BT::InputPort<double>("AscendingOvershootOffset"),
      BT::OutputPort<double>("TickDescendHeight")
  };
}

BT::NodeStatus ForkActionNode::tick()
{
  auto palletThickness = getInput<double>("PalletThickness");
  auto palletBottomPadding = getInput<double>("PalletBottomPadding");
  auto palletPose = getInput<tf::StampedTransform>("PalletPose");
  auto targetHeight = getInput<double>("TargetHeight");
  auto targetHeightOffset = getInput<double>("TargetHeightOffset");
  auto ascendingOvershootOffset = getInput<double>("AscendingOvershootOffset");

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
    forkHeightGoal.targetHeight = (abs(palletPose.value().getOrigin().getZ()
        - palletThickness.value() / 2 + palletBottomPadding.value())) * 1e3;
  }
  else if (targetHeight)
  {
    forkHeightGoal.targetHeight = targetHeight.value() * 1e3;
  }
  else if (targetHeightOffset)
  {
    auto currentHeight = ros::topic::waitForMessage<std_msgs::Float64>(
        "/afl/fork_height", ros::Duration(10));
    if (!currentHeight)
    {
      ROS_ERROR_NAMED("AFL", "[afl_behavior_tree | ForkActionNode] "
          "missing height sensor data when given TargetHeightOffset");
      return BT::NodeStatus::FAILURE;
    }
    forkHeightGoal.targetHeight =
        (currentHeight->data + targetHeightOffset.value()) * 1e3;
  }

  forkHeightGoal.ascendingOvershootOffset =
      ascendingOvershootOffset.value_or(0.0) * 1e3;

  ROS_INFO_STREAM_NAMED("[AFL|afl_behavior_tree|ForkActionNode]",
      this->name() << " setting fork to height " << forkHeightGoal.targetHeight
      << " mm, with overshoot = " << forkHeightGoal.ascendingOvershootOffset
      << " mm");

  setOutput("TickDescendHeight", forkHeightGoal.targetHeight * 1e-3);

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
