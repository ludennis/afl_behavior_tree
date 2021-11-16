#ifndef _MOVE_ACTION_NODE_
#define _MOVE_ACTION_NODE_

#include <string>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

#include <aflctrl_msgs/setReleaseBrake.h>
#include <aflctrl_msgs/setAutoMode.h>

namespace AFL
{

class MoveActionNode : public BT::SyncActionNode
{
 public:
  MoveActionNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  BT::NodeStatus sendMoveGoal(
      const move_base_msgs::MoveBaseGoal &moveBaseGoal);

 private:
  bool ReleaseBrake();

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mActionClient;
  move_base_msgs::MoveBaseGoal mMoveBaseGoal;
  ros::NodeHandle mNodeHandle;
};

} // namespace AFL

#endif // _MOVE_ACTION_NODE_
