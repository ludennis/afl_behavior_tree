#include <raiseForkActionNode.h>

namespace AFL
{

RaiseForkActionNode::RaiseForkActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
, mActionClient("RaiseForkActionClient", true)
{}

BT::PortsList RaiseForkActionNode::providedPorts()
{
  return {
      BT::InputPort<geometry_msgs::PoseWithCovarianceStamped>("PalletPose"),
      BT::InputPort<short int>("TargetHeight") };
}

BT::NodeStatus RaiseForkActionNode::tick()
{
  afl_fork_control::setForkGoal forkGoal;

  auto palletPose = getInput<geometry_msgs::PoseWithCovarianceStamped>(
      "PalletPose");
  auto targetHeight = getInput<short int>("TargetHeight");

  if (!palletPose && !targetHeight)
  {
    throw BT::RuntimeError(
        "Missing required input PalletPose and TargetHeight: ",
        palletPose.error());
    return BT::NodeStatus::FAILURE;
  }

  forkGoal.set_height =
      (palletPose) ? abs(palletPose.value().pose.pose.position.z * 1e3) :
      targetHeight.value();

  ROS_INFO_STREAM_NAMED("[AFL|afl_behavior_tree|RaiseForkActionNode]",
      this->name() << " setting for to height " << forkGoal.set_height
      << " mm");

  return sendForkGoal(forkGoal);
}

BT::NodeStatus RaiseForkActionNode::sendForkGoal(
    const afl_fork_control::setForkGoal &forkGoal)
{
  this->mActionClient.waitForServer();
  this->mActionClient.sendGoal(forkGoal);
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
