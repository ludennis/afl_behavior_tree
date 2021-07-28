#ifndef _AFL_BEHAVIOR_H_
#define _AFL_BEHAVIOR_H_

#include "shared_class.h"

namespace AFL
{
  class ForkLower : public Forkctrl
  {
    public:
      ForkLower(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };

  class NormalDetection : public ExtendedNode
  {
    public:
      NormalDetection(const std::string& name, const NodeConfiguration& config);
      BT::NodeStatus tick() override;

    protected:
      ros::NodeHandle n;
  };
}

#endif // _AFL_BEHAVIOR_
