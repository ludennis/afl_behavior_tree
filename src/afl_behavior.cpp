#include "afl_behavior.h"

namespace AFL
{
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
