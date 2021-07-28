#ifndef _RAISE_FORK_H_
#define _RAISE_FORK_H_

#include <shared_class.h>

namespace AFL
{
  class RaiseFork : public Forkctrl
  {
    public:
      RaiseFork(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };
} // namsespace AFL

#endif // _RAISE_FORK_H_
