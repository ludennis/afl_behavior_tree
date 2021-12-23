#ifndef _MOVE_TO_TARGET_GLOBAL_POSE_H_
#define _MOVE_TO_TARGET_GLOBAL_POSE_H_

#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <mbf_msgs/MoveBaseAction.h>

namespace AFL
{

class MoveToTargetGlobalPose : public BT::SyncActionNode
{
 public:
  MoveToTargetGlobalPose(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  bool SendMoveGoal(const mbf_msgs::MoveBaseGoal &goal);
 private:
  actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> mActionClient;
  mbf_msgs::MoveBaseGoal mMoveBaseFlexGoal;
};

} // namespace AFL

#endif // _MOVE_TO_TARGET_GLOBAL_POSE_H_
