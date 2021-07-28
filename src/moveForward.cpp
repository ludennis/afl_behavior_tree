#include <moveForward.h>

namespace AFL
{
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
} // namespace AFL
