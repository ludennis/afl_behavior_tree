#ifndef _LOWER_FORK_H_
#define _LOWER_FORK_H_

#include <shared_class.h>

namespace AFL
{
  class LowerFork : public Forkctrl
  {
    public:
      LowerFork(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };
} // namespace AFL
#endif // _LOWER_FORK_H_
