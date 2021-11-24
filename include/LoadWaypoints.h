#ifndef _LOAD_WAYPOINTS_H_
#define _LOAD_WAYPOINTS_H_

#include <fstream>
#include <sstream>

#include <ros/ros.h>

#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/PoseArray.h>

namespace AFL
{

class LoadWaypoints : public BT::SyncActionNode
{
 public:
  LoadWaypoints(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

 private:
  geometry_msgs::PoseArray mWaypoints;
  ros::NodeHandle mNodeHandle;
  std::string mWaypointsFilePath;
};

} // namespace AFL

#endif // _LOAD_WAYPOINTS_H_
