#ifndef _WAIT_FOR_PALLET_DETECTION_H_
#define _WAIT_FOR_PALLET_DETECTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace AFL
{
  class WaitForPalletDetection : public BT::SyncActionNode
  {
   public:
    WaitForPalletDetection(const std::string &name,
        const BT::NodeConfiguration &config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

   private:
    ros::NodeHandle mNodeHandle;
    BT::Optional<std::string> mPalletTfName;
    BT::Optional<std::string> mRobotTfName;
  };
} // namespace AFL

#endif // _WAIT_FOR_PALLET_DETECTION_H_
