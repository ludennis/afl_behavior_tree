#include <moveBackward.h>

namespace AFL
{
  MoveBackward::MoveBackward(const std::string& name, const NodeConfiguration& config)
  : Navigation(name, config)
  {
    this->object_pose = new float[7]; // x, y, Quaternion
  }

  BT::PortsList MoveBackward::providedPorts()
  {
    return { InputPort<GoalPose>("message") };
  }

  BT::NodeStatus MoveBackward::tick()
  {
    publishBehaviorState();
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode initializing") ;

    // main implementation
    ros::param::get("~behavior_tree/MoveBackward/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/MoveBackward/Covariance_threshold", this->threshold);

    // Get height from port
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveBackward pose " <<
        this->Pose.value().poseback.position.x ) ;
    this->Pose = getInput<GoalPose>("message");

    // Check if optional is valid. If not, throw its error
    if (!this->Pose)
    {
      throw BT::RuntimeError("missing required input [message]: ", this->Pose.error());
    }

    //set Backward direction Mode
    this->setBackwardMode(true);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = this->Pose.value().posefront.position.x;
    goal.target_pose.pose.position.y = this->Pose.value().posefront.position.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.header.frame_id = "map";
    //inverse the orientation of goal
    tf2::Quaternion q_rot, q_orig, q_new;
    tf2::convert(this->Pose.value().poseback.orientation , q_orig);
    double r = 0, p = 0, y = 3.14159;
    q_rot.setRPY(r, p, y);
    q_new = q_rot*q_orig;
    q_new.normalize();
    tf2::convert(q_new, goal.target_pose.pose.orientation);

    this->setReleaseBrake();
    ros::Duration(0.5).sleep();
    this->clearCostmaps();
    sendGoal(goal);

    return isBehaviorFinished();
  }
} // namespace AFL
