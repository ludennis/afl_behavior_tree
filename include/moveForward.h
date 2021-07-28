#ifndef _MOVE_FORWARD_H_
#define _MOVE_FORWARD_H_

#include <shared_class.h>

namespace AFL
{
  class MoveForward : public Navigation
  {
    public:
      MoveForward(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      void PalletFrontEndChecker();
      void opticalSensorCallback(const std_msgs::Bool arrival);
      Optional<GoalPose> Pose;
      InsertionController insertion_controller;
      bool isPalletFrontEndArrival;
  };
} // namespace AFL
#endif // _MOVE_FORWARD_H_
