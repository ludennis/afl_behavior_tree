#ifndef _SLEEP_NODE_H_
#define _SLEEP_NODE_H_

#include <ros/ros.h>

#include <behaviortree_cpp_v3/action_node.h>

namespace AFL
{

class SleepNode : public BT::SyncActionNode
{
 public:
  SleepNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
 private:
  int mSleepTime;
};

} // namespace AFL

#endif // _SLEEP_NODE_H_
