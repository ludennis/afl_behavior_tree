#include <moveBackward.h>

namespace AFL
{

MoveBackward::MoveBackward(const std::string& name, const NodeConfiguration& config)
: BT::SyncActionNode(name, config)
, mActionClient(name, true)
{
  mCurrentPoseTopic = getInput<std::string>("CurrentPoseTopic");
  mTargetDistance = getInput<short int>("TargetDistance");
}

BT::PortsList MoveBackward::providedPorts()
{
  return { InputPort<std::string>("CurrentPoseTopic"),
      InputPort<short int>("TargetDistance")
  };
}

BT::NodeStatus MoveBackward::tick()
{
  auto currentPose =
      ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
          mCurrentPoseTopic.value_or("empty"), ros::Duration(10));

  // set goal to be behind itself
  if (currentPose)
  {
    mMoveBaseGoal.target_pose.pose = currentPose->pose.pose;
    mMoveBaseGoal.target_pose.pose.position.x += mTargetDistance.value_or(0);
    mMoveBaseGoal.target_pose.header.frame_id = "map";

    return sendMoveGoal(mMoveBaseGoal);
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MoveBackward::sendMoveGoal(
    const move_base_msgs::MoveBaseGoal &moveBaseGoal)
{
  mActionClient.waitForServer();
  mActionClient.sendGoal(moveBaseGoal);
  bool success = mActionClient.waitForResult(ros::Duration(25));

  if (success)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}
} // namespace AFL
