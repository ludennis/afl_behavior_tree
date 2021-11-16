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
      BT::InputPort<double>("TargetPoseOffset"),
  };
}

BT::NodeStatus MoveActionNode::tick()
{
  auto targetPose = getInput<tf::StampedTransform>("TargetPose");
  auto targetPoseOffset = getInput<double>("TargetPoseOffset");

  if (!targetPose)
  {
    throw BT::RuntimeError(
        "Missing required input TargetPose: ", targetPose.error());
    return BT::NodeStatus::FAILURE;
  }

  tf::Matrix3x3 m(targetPose->getRotation());

  mMoveBaseGoal.target_pose.pose.position.x = targetPose->getOrigin().getX() +
      m[0][0] * targetPoseOffset.value_or(0.0);
  mMoveBaseGoal.target_pose.pose.position.y = targetPose->getOrigin().getY() +
      m[1][0] * targetPoseOffset.value_or(0.0);
  mMoveBaseGoal.target_pose.pose.orientation.z = targetPose->getRotation().getZ();
  mMoveBaseGoal.target_pose.pose.orientation.w = targetPose->getRotation().getW();
  mMoveBaseGoal.target_pose.header.frame_id = "map";

  ROS_INFO_STREAM_NAMED("AFL",
      "[afl_behavior_tree] Sending move base goal: " << mMoveBaseGoal.target_pose);

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

  if (!ReleaseBrake())
    return BT::NodeStatus::FAILURE;

  bool success = this->mActionClient.waitForResult(ros::Duration(25));
  if (success)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}

bool MoveActionNode::ReleaseBrake()
{
  auto releaseBrakeClient =
      this->mNodeHandle.serviceClient<aflctrl_msgs::setReleaseBrake>(
      "/afl/setReleaseBrake");

  auto autoModeClient =
      this->mNodeHandle.serviceClient<aflctrl_msgs::setAutoMode>(
      "/afl/setAutoMode");

  aflctrl_msgs::setReleaseBrake releaseBrakeMessage;
  aflctrl_msgs::setAutoMode autoModeMessage;
  autoModeMessage.request.mode = true;

  if (!releaseBrakeClient.call(releaseBrakeMessage))
  {
    ROS_ERROR("[afl_behavior_tree] releaseBrakeClient error");
    return false;
  }

  if (!autoModeClient.call(autoModeMessage))
  {
    ROS_ERROR("[afl_behavior_tree] setAutoModeClient error");
    return false;
  }

  ROS_INFO_STREAM("[afl_behavior_tree|ReleaseBrakeNode] result: " <<
      releaseBrakeMessage.response.result << ", " <<
      autoModeMessage.response.result);

  return true;
}

} // namespace AFL
