#ifndef _DETECTION_CLIENT_H_
#define _DETECTION_CLIENT_H_

#include <string>

#include <ros/ros.h>

#include <behaviortree_cpp_v3/action_node.h>

#include <afl_detection/ActivateDetection.h>
#include <afl_detection/DeactivateDetection.h>


namespace AFL
{

class DetectionClient : public BT::SyncActionNode
{
 public:
  DetectionClient(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

 private:
  ros::NodeHandle mNodeHandle;
};

} // namespace AFL

#endif // _DETECTION_CLIENT_H_
