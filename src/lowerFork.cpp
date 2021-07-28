#include <lowerFork.h>

namespace AFL
{
  LowerFork::LowerFork(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

  BT::PortsList LowerFork::providedPorts()
  {
    return { InputPort<double>("Height") };
  }

  BT::NodeStatus LowerFork::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;
    ros::param::get("~behavior_tree/DropFork/goal_completion_timeout", this->duration);
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode starting");

    auto lay_height = getInput<double>("Height");
    if (!lay_height)
    {
      throw BT::RuntimeError("missing required input [message]: ", lay_height.error());
    }
    height.set_height = lay_height.value() - 20;
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " get height " << height.set_height);

    sendHeight(height);

    return isBehaviorFinished();
  }
} // namespace AFL
