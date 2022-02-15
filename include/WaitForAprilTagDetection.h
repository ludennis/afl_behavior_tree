#ifndef _WAIT_FOR_APRIL_TAG_DETECTION_H_
#define _WAIT_FOR_APRIL_TAG_DETECTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PoseStamped.h>

namespace AFL
{

class WaitForAprilTagDetection : public BT::SyncActionNode
{
 public:
  WaitForAprilTagDetection(const std::string &name,
      const BT::NodeConfiguration &config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

 private:
  ros::NodeHandle mNodeHandle;
  BT::Optional<std::string> mAprilTagTfName;
  BT::Optional<std::string> mMapTfName;
};

} // namespace AFL

#endif // _WAIT_FOR_APRIL_TAG_DETECTION_H_
