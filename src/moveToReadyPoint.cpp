#include <moveToReadyPoint.h>

namespace AFL
{
  MoveToReadyPoint::MoveToReadyPoint(const std::string& name, const NodeConfiguration& config)
  : Navigation(name, config)
  {}

  BT::PortsList MoveToReadyPoint::providedPorts()
  {
    return { InputPort<Pose2D>("ReadyPoint") };
  }

  BT::NodeStatus MoveToReadyPoint::tick()
  {
    publishBehaviorState();
    ros::param::get("~behavior_tree/MoveToReadyPoint/goal_completion_timeout", this->duration);
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: "
        << this->name() << " actionNode initializing") ;

    move_base_msgs::MoveBaseGoal goal;
    auto RP_pose = getInput<Pose2D>("ReadyPoint");
    if (!RP_pose)
    {
      throw BT::RuntimeError("missing required input [message]: ", RP_pose.error());
    }

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = Pose_theta2quaternion(RP_pose.value());

    sendGoal(goal);
    return isBehaviorFinished();
  }

  geometry_msgs::Pose MoveToReadyPoint::Pose_theta2quaternion(Pose2D pose)
  {
    tf2::Quaternion q;
    geometry_msgs::Pose output;
    output.position.x  = pose.x;
    output.position.y  = pose.y;
    std::cout << "Pose: "<< pose.y << std::endl;
    q.setRPY(0, 0, pose.theta);
    tf2::convert(q, output.orientation);

    return output;
  }
} // namespace AFL
