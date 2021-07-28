#include "afl_behavior.h"

namespace AFL
{
  ForkHigher::ForkHigher(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

  BT::PortsList ForkHigher::providedPorts()
  {
    return { InputPort<GoalPose>("message") };
  }

  BT::NodeStatus ForkHigher::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;

    int higher_distance;
    ros::param::get("~behavior_tree/forkhigher/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/forkhigher/higher_distance", higher_distance);
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

  MoveForward::MoveForward(const std::string& name, const NodeConfiguration& config)
  : Navigation(name, config)
  {
    this->object_pose = new float[7]; // x, y, Quaternion
  }

  BT::PortsList MoveForward::providedPorts()
  {
    return { InputPort<GoalPose>("message") };
  }

  BT::NodeStatus MoveForward::tick()
  {
    publishBehaviorState();
    //thread checks if the pallet front end arrives
    std::thread t1(&MoveForward::PalletFrontEndChecker, this);

    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode initializing") ;

    // main implementation
    ros::param::get("~behavior_tree/moveforward/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/moveforward/Covariance_threshold", this->threshold);

    // Get height from port
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] forward pose " <<
        this->Pose.value().poseback.position.x);
    this->Pose = getInput<GoalPose>("message");

    // Check if optional is valid. If not, throw its error
    if (!this->Pose)
    {
      throw BT::RuntimeError("missing required input [message]: ", this->Pose.error());
    }

    //set back pose
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = this->Pose.value().poseback;
    goal.target_pose.header.frame_id = "map";
    this->setReleaseBrake();

    //set Backward direction Mode
    this->setBackwardMode(false);
    ros::Duration(0.5).sleep();
    this->clearCostmaps();

    sendGoal(goal);
    this->ac.waitForResult(ros::Duration(this->duration));

    //insertion controller
    if(this->ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      while(!this->isPalletFrontEndArrival)
      {
        insertion_controller.setForward(true);
      }
      insertion_controller.setForward(false);
    }
    t1.join();
    ros::Duration(1).sleep();

    return isBehaviorFinished();
  }

  void MoveForward::PalletFrontEndChecker()
  {
    ros::Subscriber optical_sensor_sub = this->nh.subscribe(
        "/afl/sick_WTB16P", 1, &MoveForward::opticalSensorCallback, this);

    this->isPalletFrontEndArrival = false;
    while(!this->isPalletFrontEndArrival)
    {
      ros::spinOnce();
    }
    optical_sensor_sub.shutdown();
  }

  void MoveForward::opticalSensorCallback(const std_msgs::Bool arrival)
  {
    if(arrival.data == true)
    {
      this->isPalletFrontEndArrival = true;
      this->ac.cancelGoal();
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveForward: "
          "Pallet front end arrives!");
    }
    this->clearCostmaps();
  }

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
