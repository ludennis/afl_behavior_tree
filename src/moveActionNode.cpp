#include <moveActionNode.h>

namespace AFL
{

MoveActionNode::MoveActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: SyncActionNode(name, config)
, mActionClient("move_base", true)
{}

BT::PortsList MoveActionNode::providedPorts()
{
  return {
      BT::InputPort<tf::StampedTransform>("TargetPose"),
      BT::InputPort<short int>("BackAwayDistance")
  };
}

BT::NodeStatus MoveActionNode::tick()
{
  auto targetPose = getInput<tf::StampedTransform>("TargetPose");
  auto backAwayDistance = getInput<short int>("BackAwayDistance");

  if (!targetPose)
  {
    throw BT::RuntimeError(
        "Missing required input TargetPose: ", targetPose.error());
    return BT::NodeStatus::FAILURE;
  }

  mMoveBaseGoal.target_pose.pose.position.x = targetPose->getOrigin().getX()
      - backAwayDistance.value_or(0);
  mMoveBaseGoal.target_pose.pose.position.y = targetPose->getOrigin().getY();
  mMoveBaseGoal.target_pose.pose.position.z = 0.0;
  mMoveBaseGoal.target_pose.header.frame_id = "map";

  return sendMoveGoal(mMoveBaseGoal);
}

BT::NodeStatus MoveActionNode::sendMoveGoal(
    const move_base_msgs::MoveBaseGoal &moveBaseGoal)
{
  ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] " << this->name() <<
      ": waiting for move base action server to start.");

  this->mActionClient.waitForServer();

  ROS_INFO_STREAM_NAMED("AFL", "[afl_behavior_tree] " << this->name() <<
      ": move base action server started, sending goal.");

  this->mActionClient.sendGoal(moveBaseGoal);

  bool success = this->mActionClient.waitForResult(ros::Duration(25));
  if (success)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}

} // namespace AFL
