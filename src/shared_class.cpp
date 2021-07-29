#include <shared_class.h>

namespace AFL
{
  void callService::setReleaseBrake()
  {
    auto releasebrake_client = this->nh.serviceClient<aflctrl_msgs::setReleaseBrake>(
        "/afl/setReleaseBrake");
    aflctrl_msgs::setReleaseBrake releasebrake_msg;
    if (releasebrake_client.call(releasebrake_msg))
      ROS_INFO_STREAM_NAMED("AFL", "[AFL Navigation] Result: " <<
          releasebrake_msg.response.result);
    else
      ROS_ERROR_STREAM_NAMED("AFL",
          "[AFL Navigation] Failed to call service setReleaseBrake");
  }

  void callService::setDeactivate()
  {
    //shut down the detection
    auto deactivate = this->nh.serviceClient<detection::DeactivateDetection>(
        "/deactivate_detection");
    detection::DeactivateDetection deactivate_msg;
    if (deactivate.call(deactivate_msg))
      ROS_INFO_STREAM_NAMED("AFL", "[AFL Navigation] Result: " <<
          deactivate_msg.response.done);
    else
      ROS_ERROR_STREAM_NAMED("AFL", "[AFL Navigation] Failed to call service "
          "DeactivateDetection");
  }

  void callService::setBackwardMode(bool mode)
  {
    auto BackMode_client =
        this->nh.serviceClient<reconfigure_client::setBackwardMode>(
            "/setBackwardMode");
    reconfigure_client::setBackwardMode BackMode_msg;
    BackMode_msg.request.mode = mode;
    if (BackMode_client.call(BackMode_msg))
      ROS_INFO_STREAM_NAMED("AFL", "[AFL Behavior] Result: " <<
          BackMode_msg.response.result);
    else
      ROS_ERROR_STREAM_NAMED("AFL",
          "[AFL Behavior] Failed to call service setBackwardMode");
  }

  void callService::clearCostmaps()
  {
    auto clear_costmaps_client = this->nh.serviceClient<std_srvs::Empty>(
        "/move_base/clear_costmaps");
    std_srvs::Empty clear_costmaps_msg;
    if (clear_costmaps_client.call(clear_costmaps_msg))
      ROS_INFO_STREAM_NAMED("AFL", "[AFL Behavior] Result: clearCostmaps success!");
    else
      ROS_ERROR_STREAM_NAMED("AFL",
          "[AFL Behavior] Failed to call service clearCostmaps");
  }

  ExtendedNode::ExtendedNode(const std::string& name, const NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
  {
    Node_finished = false;
  }

  void ExtendedNode::publishBehaviorState()
  {
    BT_state_pub = nh.advertise<std_msgs::String>("BehaviorState", 1000);

    //thread checks if the pallet front end arrives
    std::thread threadForPublish(&ExtendedNode::BehaviorPublisher, this);
    threadForPublish.detach();
  }

  void ExtendedNode::BehaviorPublisher()
  {
    std_msgs::String msg;
    ros::Rate loop_rate(5);
    while(!this->Node_finished)
    {
      msg.data = this->name();
      this->BT_state_pub.publish(msg);
      loop_rate.sleep();
    }
  }

  Navigation::Navigation(const std::string& name, const NodeConfiguration& config)
  : ac("move_base", true)
  , ExtendedNode(name, config)
  {}

  void Navigation::sendGoal(move_base_msgs::MoveBaseGoal goal)
  {
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] " << this->name() <<
        ": Waiting for move base action server to start.");
    this->ac.waitForServer(); //will wait for infinite time
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] " << this->name() <<
        ": Move base action server started, sending goal.");
    this->ac.sendGoal(goal);
  }

  BT::NodeStatus Navigation::isBehaviorFinished()
  {
    this->ac.waitForResult(ros::Duration(this->duration));
    Node_finished = true;
    if (this->ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return BT::NodeStatus::SUCCESS;
    else if (this->ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
      return BT::NodeStatus::SUCCESS;
    else
    {
      ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] "<< this->name() << ": fail");
      return BT::NodeStatus::FAILURE;
    }
  }

  Forkctrl::Forkctrl(const std::string& name, const NodeConfiguration& config)
  : ac("fork_action", true)
  , ExtendedNode(name, config)
  {}

  void Forkctrl::sendHeight(afl_fork_control::setForkGoal height)
  {
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] " << this->name() <<
        ": Waiting for move base action server to start.");
    this->ac.waitForServer(); //will wait for infinite time
    ROS_INFO_STREAM_NAMED("AFL","[afl_behavior_tree] " << this->name() <<
        ": Move base action server started, sending goal.");

    this->ac.sendGoal(height);
  }

  BT::NodeStatus Forkctrl::isBehaviorFinished()
  {
    bool finished_before_timeout =
        this->ac.waitForResult(ros::Duration(this->duration));
    Node_finished = true;
    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = this->ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());

      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
      return BT::NodeStatus::FAILURE;
    }
  }
} // namespace AFL
