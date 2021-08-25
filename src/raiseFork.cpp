#include <raiseFork.h>

namespace AFL
{
  RaiseFork::RaiseFork(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

  BT::PortsList RaiseFork::providedPorts()
  {
    return { InputPort<geometry_msgs::PoseWithCovarianceStamped>("PalletPose") };
  }

  BT::NodeStatus RaiseFork::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;

    int higher_distance;
    ros::param::get("~behavior_tree/RaiseFork/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/RaiseFork/higher_distance", higher_distance);
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode starting") ;
    // Get height from port
    this->Pose = getInput<GoalPose>("message");
    // Check if optional is valid. If not, throw its error
    if (!this->Pose)
    {
      throw BT::RuntimeError("missing required input [message]: ", this->Pose.error());
    }

    height.set_height = abs(this->Pose.value().posefront.position.z * 1e3) + higher_distance;
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " get height " << height.set_height) ;

    sendHeight(height);
    return isBehaviorFinished();
  }
} // namespace AFL
