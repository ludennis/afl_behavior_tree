#include <moveActionNode.h>

namespace AFL
{

MoveActionNode::MoveActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: SyncActionNode(name, config)
, mActionClient("RaiseForkActionClient", true)
{}

BT::PortsList MoveActionNode::providedPorts()
{
  return {
      BT::InputPort<geometry_msgs::PoseWithCovarianceStamped>("TargetPose") };
}

BT::NodeStatus MoveActionNode::tick()
{
  auto targetPose =
      getInput<geometry_msgs::PoseWithCovarianceStamped>("TargetPose");

  if (!targetPose)
  {
    throw BT::RuntimeError(
        "Missing required input TargetPose: ", targetPose.error());
    return BT::NodeStatus::FAILURE;
  }

  mMoveBaseGoal.target_pose.pose = targetPose->pose.pose;
  mMoveBaseGoal.target_pose.pose.position.z = 0.0;
  mMoveBaseGoal.target_pose.header.frame_id = "map";

  return sendMoveGoal(mMoveBaseGoal);
}

BT::NodeStatus MoveActionNode::sendMoveGoal(
    const move_base_msgs::MoveBaseGoal &moveBaseGoal)
{
  this->mActionClient.waitForServer();
  this->mActionClient.sendGoal(moveBaseGoal);
  bool success = this->mActionClient.waitForResult(ros::Duration(25));

  if (success)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}

} // namespace AFL
