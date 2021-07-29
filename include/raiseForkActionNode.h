#ifndef _RAISE_FORK_ACTION_H_
#define _RAISE_FORK_ACTION_H_

#include <string>

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <afl_fork_control/setForkAction.h>

namespace AFL
{

class RaiseForkActionNode : public BT::SyncActionNode
{
 public:
  RaiseForkActionNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  BT::NodeStatus sendForkGoal(const afl_fork_control::setForkGoal &forkGoal);

 private:
  actionlib::SimpleActionClient<afl_fork_control::setForkAction> mActionClient;
};

} // namespace AFL

#endif // _RAISE_FORK_ACTION_H_

