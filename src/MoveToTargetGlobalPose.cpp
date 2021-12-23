#include <MoveToTargetGlobalPose.h>

namespace AFL
{

MoveToTargetGlobalPose::MoveToTargetGlobalPose(
    const std::string &name, const BT::NodeConfiguration &config)
: SyncActionNode(name, config)
, mActionClient("move_base_flex", true)
{}

BT::PortsList MoveToTargetGlobalPose::providedPorts()
{
  return {
      BT::InputPort<double>("TargetPositionX"),
      BT::InputPort<double>("TargetPositionY"),
      BT::InputPort<double>("TargetQuaternionZ"),
      BT::InputPort<double>("TargetQuaternionW"),
      BT::InputPort<std::string>("Controller"),
      BT::InputPort<std::string>("Planner"),
  };
}

BT::NodeStatus MoveToTargetGlobalPose::tick()
{
  auto targetPositionX = getInput<double>("TargetPositionX");
  auto targetPositionY = getInput<double>("TargetPositionY");
  auto targetQuaternionZ = getInput<double>("TargetQuaternionZ");
  auto targetQuaternionW = getInput<double>("TargetQuaternionW");
  auto controller = getInput<std::string>("Controller");
  auto planner = getInput<std::string>("Planner");

  mMoveBaseFlexGoal.target_pose.header.frame_id = "map";
  mMoveBaseFlexGoal.target_pose.pose.position.x =
      targetPositionX.value_or(0.0);
  mMoveBaseFlexGoal.target_pose.pose.position.y =
      targetPositionY.value_or(0.0);
  mMoveBaseFlexGoal.target_pose.pose.orientation.z =
      targetQuaternionZ.value_or(0.0);
  mMoveBaseFlexGoal.target_pose.pose.orientation.w =
      targetQuaternionW.value_or(0.0);
  mMoveBaseFlexGoal.controller = controller.value_or("");
  mMoveBaseFlexGoal.planner = planner.value_or("");

  return (SendMoveGoal(mMoveBaseFlexGoal)) ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

bool MoveToTargetGlobalPose::SendMoveGoal(
    const mbf_msgs::MoveBaseGoal &goal)
{
  ROS_INFO_STREAM("[afl_behavior_tree] " << this->name() <<
      ": waiting for move_base_flex action server to start");
  this->mActionClient.waitForServer();
  ROS_INFO_STREAM("[afl_behavior_tree] " << this->name() <<
      ": move_base_flex action server started, sending goal");
  this->mActionClient.sendGoal(goal);

  return this->mActionClient.waitForResult();
}

} // namespace AFL
