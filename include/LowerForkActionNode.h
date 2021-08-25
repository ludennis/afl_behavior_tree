#ifndef _LOWER_FORK_ACTION_NODE_H_
#define _LOWER_FORK_ACTION_NODE_H_

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <afl_fork_control/setForkHeightAction.h>

namespace AFL
{

class LowerForkActionNode : public BT::SyncActionNode
{
 public:
  LowerForkActionNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  BT::NodeStatus sendForkHeightGoal(
      const afl_fork_control::setForkHeightGoal &forkHeightGoal);
 private:
  actionlib::SimpleActionClient<afl_fork_control::setForkHeightAction> mActionClient;
};

} // namespace AFL

#endif // _LOWER_FORK_ACTION_NODE_H_
