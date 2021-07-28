#ifndef _DROP_FORK_H_
#define _DROP_FORK_H_

#include <shared_class.h>

namespace AFL
{
  class DropFork : public Forkctrl
  {
    public:
      DropFork(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };
} // namespace AFL
#endif // _DROP_FORK_H_
