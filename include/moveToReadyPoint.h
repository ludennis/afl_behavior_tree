#ifndef _MOVE_TO_READY_POINT_H_
#define _MOVE_TO_READY_POINT_H_

#include <shared_class.h>

namespace AFL
{
  class MoveToReadyPoint : public Navigation
  {
    public:
      MoveToReadyPoint(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      geometry_msgs::Pose Pose_theta2quaternion(Pose2D pose);
  };
} // namespace AFL

#endif // _MOVE_TO_READY_POINT_H_
