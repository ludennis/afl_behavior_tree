#include <moveActionNode.h>

namespace AFL
{

MoveActionNode::MoveActionNode(
    const std::string &name, const BT::NodeConfiguration &config)
: SyncActionNode(name, config)
, mActionClient("move_base_flex/move_base", true)
, mWaypointsIndex(0)
{}

BT::PortsList MoveActionNode::providedPorts()
{
  return {
      BT::InputPort<tf::StampedTransform>("TargetPose"),
      BT::InputPort<double>("TargetPoseOffset"),
      BT::InputPort<geometry_msgs::PoseArray>("Waypoints"),
      BT::InputPort<std::string>("Controller"),
      BT::InputPort<std::string>("Planner"),
  };
}

BT::NodeStatus MoveActionNode::tick()
{
  auto targetPose = getInput<tf::StampedTransform>("TargetPose");
  auto targetPoseOffset = getInput<double>("TargetPoseOffset");
  auto waypoints = getInput<geometry_msgs::PoseArray>("Waypoints");
  auto controller = getInput<std::string>("Controller");
  auto planner = getInput<std::string>("Planner");

  if (targetPose)
  {
    tf::Matrix3x3 m(targetPose->getRotation());

    mMoveBaseFlexGoal.target_pose.pose.position.x = targetPose->getOrigin().getX() +
        m[0][0] * targetPoseOffset.value_or(0.0);
    mMoveBaseFlexGoal.target_pose.pose.position.y = targetPose->getOrigin().getY() +
        m[1][0] * targetPoseOffset.value_or(0.0);
    mMoveBaseFlexGoal.target_pose.pose.orientation.z = targetPose->getRotation().getZ();
    mMoveBaseFlexGoal.target_pose.pose.orientation.w = targetPose->getRotation().getW();
    mMoveBaseFlexGoal.target_pose.header.frame_id = "map";
    mMoveBaseFlexGoal.controller = controller.value_or("");
    mMoveBaseFlexGoal.planner = planner.value_or("");

    ROS_INFO_STREAM_NAMED("AFL",
        "[afl_behavior_tree] Sending move base flex goal: " << mMoveBaseFlexGoal.target_pose);
    ROS_INFO_STREAM_NAMED("AFL",
        "[afl_behavior_tree] Move base flex using controller: " << mMoveBaseFlexGoal.controller <<
        " and planner: " << mMoveBaseFlexGoal.planner);

    return (sendMoveGoal(mMoveBaseFlexGoal)) ?
        BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  else if (waypoints)
  {
    ROS_INFO("[afl_behavior_tree] Following waypoints!");

    while (mWaypointsIndex < waypoints.value().poses.size())
    {
      mMoveBaseFlexGoal.target_pose.pose = waypoints.value().poses[mWaypointsIndex];
      mMoveBaseFlexGoal.target_pose.header.frame_id = "map";
      mMoveBaseFlexGoal.controller = controller.value_or("");
      mMoveBaseFlexGoal.planner = planner.value_or("");

      if (sendMoveGoal(mMoveBaseFlexGoal))
        mWaypointsIndex++;
      else
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    throw BT::RuntimeError(
        "Missing required input TargetPose/Waypoints: ", targetPose.error());
    return BT::NodeStatus::FAILURE;
  }
}

bool MoveActionNode::sendMoveGoal(
    const mbf_msgs::MoveBaseGoal &goal)
{
  ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] " << this->name() <<
      ": waiting for move base action server to start.");

  this->mActionClient.waitForServer();

  ROS_INFO_STREAM_NAMED("AFL", "[afl_behavior_tree] " << this->name() <<
      ": move base action server started, sending goal.");

  this->mActionClient.sendGoal(goal);

  if (!ReleaseBrake())
    return false;

  bool success = this->mActionClient.waitForResult();

  return success;
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
