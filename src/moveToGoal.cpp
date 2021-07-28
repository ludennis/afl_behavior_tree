#include <moveToGoal.h>

namespace AFL
{
  MoveToGoal::MoveToGoal(const std::string& name, const NodeConfiguration& config) :
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

  BT::PortsList MoveToGoal::providedPorts()
  {
    return { OutputPort<GoalPose>("Pose") };
  }

  BT::NodeStatus MoveToGoal::tick()
  {
    publishBehaviorState();
    this->getParam();
    this->detected_front = nh.subscribe(
        this->pose_front_topic, 1, &MoveToGoal::posefrontCallback, this);
    this->detected_back = nh.subscribe(
        this->pose_back_topic, 1, &MoveToGoal::posebackCallback, this);

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

  void MoveToGoal::getParam()
  {
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] Behavior: " << this->name() <<
        " actionNode initializing") ;

    // main implementation
    ros::param::get("~behavior_tree/MoveToGoal/detected_object_topic", this->pose_front_topic);
    ros::param::get("~behavior_tree/moveforward/detected_object_topic", this->pose_back_topic);
    ros::param::get("~behavior_tree/MoveToGoal/goal_completion_timeout", this->duration);
    ros::param::get("~behavior_tree/MoveToGoal/Covariance_threshold", this->threshold);
    ros::param::get("~behavior_tree/MoveToGoal/pallet_height", this->pallet_height);
  }

  void MoveToGoal::posefrontCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
  {
    enum diagonal {X=0, Y=7, Z=14, RAW=21, PITCH=28, YAW=35};
    auto total_cov = pose->pose.covariance[X] + pose->pose.covariance[Y] +
        pose->pose.covariance[Z];

    if(total_cov >= this->threshold)
    {
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveToGoal: "
          "Front pose covariance below threshold.");
      return ;
    }
    else
    {
      geometry_msgs::Pose temp_pose;
      temp_pose = pose->pose.pose;
      if( !this->poseChecker(temp_pose, this->buffer))
      {
        ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveToGoal: Front pose not stable");
        this->buffer = pose->pose.pose;
        return;
      }
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveToGoal: Get front pose.");
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

  void MoveToGoal::posebackCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
  {
    enum diagonal {X=0, Y=7, Z=14, RAW=21, PITCH=28, YAW=35};
    auto total_cov = pose->pose.covariance[X] + pose->pose.covariance[Y] +
        pose->pose.covariance[Z];
    if( total_cov >= this->threshold )//|| this->front_isPoseValid == false)
    {
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveToGoal: "
          "Back pose covariance below threshold.");
      return ;
    }
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] MoveToGoal: Get back pose.");
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
  bool MoveToGoal::poseChecker(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
  {
    double pos_threshold;
    double ori_threshold;
    ros::param::get("~behavior_tree/MoveToGoal/pos_osci_threshold", pos_threshold);
    ros::param::get("~behavior_tree/MoveToGoal/ori_osci_threshold", ori_threshold);
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
} // namespace AFL
