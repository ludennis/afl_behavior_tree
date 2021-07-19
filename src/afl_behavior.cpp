#include "afl_behavior.h"

namespace AFL
{
  Move2RP::Move2RP(const std::string& name, const NodeConfiguration& config)
  : Navigation(name, config)
  {}

  BT::PortsList Move2RP::providedPorts()
  {
    return { InputPort<Pose2D>("ReadyPoint") };
  }

  BT::NodeStatus Move2RP::tick()
  {
    publishBehaviorState();
    ros::param::get("~behavior_tree/move2RP/goal_completion_timeout", this->duration);
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

  geometry_msgs::Pose Move2RP::Pose_theta2quaternion(Pose2D pose)
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

  Move2Goal::Move2Goal(const std::string& name, const NodeConfiguration& config) :
    Navigation(name, config)
  {
    this->front_isPoseValid = false;
    this->back_isPoseValid = false;
    this->buffer.position.x = 0;
    this->buffer.position.y = 0;
    this->buffer.position.z = 0;
    this->buffer.orientation.x = 0;
    this->buffer.orientation.y = 0;
    this->buffer.orientation.z = 0;
    this->buffer.orientation.w = 1;
  }

  BT::PortsList Move2Goal::providedPorts()
  {
    return { OutputPort<GoalPose>("Pose") };
  }

  BT::NodeStatus Move2Goal::tick()
  {
    publishBehaviorState();
    this->getParam();
    this->detected_front = nh.subscribe(
        this->pose_front_topic, 1, &Move2Goal::posefrontCallback, this);
    this->detected_back = nh.subscribe(
        this->pose_back_topic, 1, &Move2Goal::posebackCallback, this);

    while( !(this->front_isPoseValid & this->back_isPoseValid) )
    {
      ros::spinOnce();
    }
    this->detected_front.shutdown();
    this->detected_back.shutdown();

    setOutput("targetPose", this->Pose); // bt port

    //set goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = front_pose;

    goal.target_pose.header.frame_id = "map";
    this->setReleaseBrake();

    sendGoal(goal);
    return isBehaviorFinished();
  }

  void Move2Goal::getParam()
  {
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode initializing") ;

    // main implementation
    ros::param::get("~behavior_tree/move2goal/detected_object_topic", this->pose_front_topic);
    ros::param::get("~behavior_tree/moveforward/detected_object_topic", this->pose_back_topic);
    ros::param::get("~behavior_tree/move2goal/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/move2goal/Covariance_threshold", this->threshold);
    ros::param::get("~behavior_tree/move2goal/pallet_height", this->pallet_height);
  }

  void Move2Goal::posefrontCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
  {
    enum diagonal {X=0, Y=7, Z=14, RAW=21, PITCH=28, YAW=35};
    auto total_cov = pose->pose.covariance[X] + pose->pose.covariance[Y] +
        pose->pose.covariance[Z];

    if(total_cov >= this->threshold)
    {
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Move2Goal: "
          "Front pose covariance below threshold.");
      return ;
    }
    else
    {
      geometry_msgs::Pose temp_pose;
      temp_pose = pose->pose.pose;
      if( !this->poseChecker(temp_pose, this->buffer))
      {
        ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Move2Goal: Front pose not stable");
        this->buffer = pose->pose.pose;
        return;
      }
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Move2Goal: Get front pose.");
      tf2::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y,
          pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);

      //get front pose
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      tf2::Quaternion q1;
      q1.setRPY(0, 0, pitch);
      front_pose = pose->pose.pose;
      tf2::convert(q1, front_pose.orientation);

      //set behavior tree port <GoalPose>
      this->Pose.posefront.position = pose->pose.pose.position;
      this->Pose.posefront.position.z = this->pallet_height; //fix height
      tf2::convert(q1, this->Pose.posefront.orientation);

      this->front_isPoseValid = true;
    }
  }

  void Move2Goal::posebackCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
  {
    enum diagonal {X=0, Y=7, Z=14, RAW=21, PITCH=28, YAW=35};
    auto total_cov = pose->pose.covariance[X] + pose->pose.covariance[Y] +
        pose->pose.covariance[Z];
    if( total_cov >= this->threshold )//|| this->front_isPoseValid == false)
    {
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Move2Goal: "
          "Back pose covariance below threshold.");
      return ;
    }
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Move2Goal: Get back pose.");
    tf2::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y,
        pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);

    //set behavior tree port <GoalPose>
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf2::Quaternion q1;
    q1.setRPY(0, 0, pitch);

    this->Pose.poseback.position = pose->pose.pose.position;
    this->Pose.poseback.position.z = this->pallet_height; //fix height
    tf2::convert(q1, this->Pose.poseback.orientation);

    this->back_isPoseValid = true;
  }

  // check if the pose is oscillating
  bool Move2Goal::poseChecker(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
  {
    double pos_threshold;
    double ori_threshold;
    ros::param::get("~behavior_tree/move2goal/pos_osci_threshold", pos_threshold);
    ros::param::get("~behavior_tree/move2goal/ori_osci_threshold", ori_threshold);
    double vector[7]; 
    vector[0] = pose1.position.x - pose2.position.x;
    vector[1] = pose1.position.y - pose2.position.y;
    vector[2] = pose1.position.z - pose2.position.z;
    vector[3] = pose1.orientation.x - pose2.orientation.x;
    vector[4] = pose1.orientation.y - pose2.orientation.y;
    vector[5] = pose1.orientation.z - pose2.orientation.z;
    vector[6] = pose1.orientation.w - pose2.orientation.w;

    double accum_p = 0;
    double accum_o = 0;
    for(int i=0; i<3; i++)
    {
      accum_p = vector[i] * vector[i];
    }
    accum_p = sqrt(accum_p);

    for(int i=3; i<7; i++)
    {
      accum_o = vector[i] * vector[i];
    }
    accum_o = sqrt(accum_o);

    if(accum_p < pos_threshold && accum_o < ori_threshold)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  Move2Height::Move2Height(const std::string& name, const NodeConfiguration& config)
  : Forkctrl(name, config)
  {}

  BT::PortsList Move2Height::providedPorts()
  {
    return { InputPort<GoalPose>("message") };
  }

  BT::NodeStatus Move2Height::tick()
  {
    publishBehaviorState();
    afl_fork_control::setForkGoal height;

    double offset;
    ros::param::get("~behavior_tree/move2height/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/move2height/height_offset", offset);
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode starting");

    // Get height from port
    this->Pose = getInput<GoalPose>("message");

    // Check if optional is valid. If not, throw its error
    if (!this->Pose)
    {
      throw BT::RuntimeError("missing required input [message]: ", this->Pose.error());
    }

    height.set_height = abs(this->Pose.value().posefront.position.z * 1e3 + offset);
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() << 
        " get height " << height.set_height) ;

    sendHeight(height);
    return isBehaviorFinished();
  }

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
