#ifndef _RAISE_FORK_ACTION_H_
#define _RAISE_FORK_ACTION_H_

#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <std_msgs/Float64.h>
#include <afl_fork_control/setForkHeightAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

namespace AFL
{

class ForkActionNode : public BT::SyncActionNode
{
 public:
  ForkActionNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  BT::NodeStatus sendForkHeightGoal(
      const afl_fork_control::setForkHeightGoal &forkHeightGoal);

 private:
  actionlib::SimpleActionClient<afl_fork_control::setForkHeightAction> mActionClient;
};

} // namespace AFL

#endif // _RAISE_FORK_ACTION_H_

