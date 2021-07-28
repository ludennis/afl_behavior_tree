#include "afl_behavior.h"

namespace AFL
{
  ForkDown::ForkDown(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

   BT::PortsList ForkDown::providedPorts()
   {
     return { InputPort<double>("Height") };
   }

  BT::NodeStatus ForkDown::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;
    ros::param::get("~behavior_tree/forkdown/goal_completion_timeout", this->duration);
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
      " actionNode starting") ;

    auto lay_height = getInput<double>("Height");
    if (!lay_height)
    {
      throw BT::RuntimeError("missing required input [message]: ", lay_height.error());
    }
    height.set_height = lay_height.value();
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " get height " << height.set_height);

    // setDeactivate();
    sendHeight(height);
    return isBehaviorFinished();
  }

  ForkLower::ForkLower(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

  BT::PortsList ForkLower::providedPorts()
  {
    return { InputPort<double>("Height") };
  }

  BT::NodeStatus ForkLower::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;
    ros::param::get("~behavior_tree/forkdown/goal_completion_timeout", this->duration);
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

  NormalDetection::NormalDetection(const std::string& name, const NodeConfiguration& config)
  : ExtendedNode(name, config)
  {}

  BT::NodeStatus NormalDetection::tick()
  {
    publishBehaviorState();
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
      " actionNode initializing") ;
    auto activate_normal = this->n.serviceClient<detection::ActivateNormalDetection>(
      "/activate_normal_detection");
    detection::ActivateNormalDetection activateNormal_msg;
    if (activate_normal.call(activateNormal_msg))
    {
      ROS_INFO_STREAM_NAMED("AFL", "[AFL Navigation] Result: " <<
          activateNormal_msg.response.done);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("AFL", "[AFL Navigation] Failed to call service "
          "ActivateNormalDetection");
      return BT::NodeStatus::FAILURE;
    }
  }
}
