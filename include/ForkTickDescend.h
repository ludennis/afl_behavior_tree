#ifndef _FORK_TICK_DESCEND_H_
#define _FORK_TICK_DESCEND_H_

#include <ros/ros.h>

#include <behaviortree_cpp_v3/action_node.h>

#include <std_msgs/Float64.h>
#include <aflctrl_msgs/setForkDown.h>
#include <aflctrl_msgs/setForkStop.h>

namespace AFL
{

class ForkTickDescend : public BT::SyncActionNode
{
 public:
  ForkTickDescend(const std::string &name,
      const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick();
  void TickDescend();
 private:
  ros::NodeHandle mNodeHandle;
};

} // namespace AFL

#endif // _FORK_TICK_DESCEND_H_
