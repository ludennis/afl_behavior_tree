#ifndef _MOVE_ACTION_NODE_
#define _MOVE_ACTION_NODE_

#include <string>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <tf/transform_datatypes.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <aflctrl_msgs/setReleaseBrake.h>
#include <aflctrl_msgs/setAutoMode.h>
#include <geometry_msgs/PoseArray.h>

namespace AFL
{

class MoveActionNode : public BT::SyncActionNode
{
 public:
  MoveActionNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool sendMoveGoal(const mbf_msgs::MoveBaseGoal &goal);

 private:
  bool ReleaseBrake();

  actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> mActionClient;
  mbf_msgs::MoveBaseGoal mMoveBaseFlexGoal;
  ros::NodeHandle mNodeHandle;

  int mWaypointsIndex;
};

} // namespace AFL

#endif // _MOVE_ACTION_NODE_
