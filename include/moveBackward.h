#ifndef _MOVE_BACKWARD_H_
#define _MOVE_BACKWARD_H_

#include <shared_class.h>

namespace AFL
{
  class MoveBackward : public Navigation
  {
    public:
      MoveBackward(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      Optional<GoalPose> Pose;
  };
} // namespace AFL
#endif // _MOVE_BACKWARD_H_
