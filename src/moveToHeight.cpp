#include <moveToHeight.h>

namespace AFL
{
  MoveToHeight::MoveToHeight(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

  BT::PortsList MoveToHeight::providedPorts()
  {
    return { InputPort<geometry_msgs::PoseWithCovarianceStamped>(
        "PalletPose") };
  }

  BT::NodeStatus MoveToHeight::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;

    auto palletPose = getInput<geometry_msgs::PoseWithCovarianceStamped>(
        "PalletPose");

    if (!palletPose)
    {
      throw BT::RuntimeError("Missing required input [PalletPose]: ",
          palletPose.error());
    }
    height.set_height =
        abs(palletPose.value().pose.pose.position.z * 1e3);
    ROS_INFO_STREAM_NAMED("[AFL|afl_behavior_tree|MoveToHeight]",
        this->name() << " setting for to height " << height.set_height
        << " mm");

    sendHeight(height);
    return isBehaviorFinished();
  }
} // namespace AFL
