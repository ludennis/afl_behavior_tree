#ifndef _MOVE_BACKWARD_H_
#define _MOVE_BACKWARD_H_

#include <string>

#include <shared_class.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <behaviortree_cpp_v3/action_node.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace AFL
{
class MoveBackward : public BT::SyncActionNode
{
  public:
    MoveBackward(const std::string& name, const BT::NodeConfiguration &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    BT::NodeStatus sendMoveGoal(
        const move_base_msgs::MoveBaseGoal &moveBaseGoal);

  private:
    Optional<std::string> mCurrentPoseTopic;
    Optional<short int> mTargetDistance;
    move_base_msgs::MoveBaseGoal mMoveBaseGoal;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mActionClient;
};
} // namespace AFL
#endif // _MOVE_BACKWARD_H_
