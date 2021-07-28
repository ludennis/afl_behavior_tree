/***************************************************************
*****This cpp file defined the interfaces for behavior node*****
***************************************************************/

#ifndef _SHARED_CLASS_H_
#define _SHARED_CLASS_H_

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <behaviortree_cpp_v3/action_node.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <afl_fork_control/setForkAction.h>
#include <aflctrl_msgs/setReleaseBrake.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include "reconfigure_client/setBackwardMode.h"

#include "detection/ActivateNormalDetection.h"
#include "detection/ActivateRallyDetection.h"
#include "detection/DeactivateDetection.h"
#include "fork_insertion_controller.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <thread>


using namespace BT;

//Template specialization
struct Pose2D{ double x, y, theta;};
template <> inline Pose2D BT::convertFromString(StringView str)
{
  auto parts = splitString(str, ';');
  if (parts.size() != 3)
  {
    throw RuntimeError("invalid input)");
  }
  else{
    Pose2D output;
    output.x   = convertFromString<double>(parts[0]);
    output.y   = convertFromString<double>(parts[1]);
    output.theta   = convertFromString<double>(parts[2]);
    return output;
  }
}

namespace AFL
{
  class GoalPose
  {
    public:
      geometry_msgs::Pose posefront;
      geometry_msgs::Pose poseback;
  };

  class callService
  {
    protected:
      ros::NodeHandle nh;
      void setReleaseBrake();
      void setDeactivate();
      void setBackwardMode(bool mode);
      void clearCostmaps();
      //add service call function
  };

  class ExtendedNode : public BT::SyncActionNode, public callService
  {
    public:
      ExtendedNode(const std::string& name, const NodeConfiguration& config);

    protected:
      void publishBehaviorState();
      void BehaviorPublisher();

      ros::Publisher BT_state_pub;
      bool Node_finished;
      float duration;
  };

  class Navigation : public ExtendedNode
  {
    public:
      Navigation(const std::string& name, const NodeConfiguration& config);

    protected:
      void sendGoal(move_base_msgs::MoveBaseGoal goal);
      BT::NodeStatus isBehaviorFinished();

      float threshold;
      float *object_pose;
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
  };

  class Forkctrl : public ExtendedNode
  {
    public:
      Forkctrl(const std::string& name, const NodeConfiguration& config);

    protected:
      void sendHeight(afl_fork_control::setForkGoal height);
      BT::NodeStatus isBehaviorFinished();

      Optional<GoalPose> Pose;
      actionlib::SimpleActionClient<afl_fork_control::setForkAction> ac;
  };
}

#endif // _SHARED_CLASS_H_
