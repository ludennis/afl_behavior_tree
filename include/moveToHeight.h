#ifndef _MOVE_TO_HEIGHT_H_
#define _MOVE_TO_HEIGHT_H_

#include <shared_class.h>

namespace AFL
{
  class MoveToHeight : public Forkctrl
  {
    public:
      MoveToHeight(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };
} // namespace AFL
#endif // _MOVE_TO_HEIGHT_H_
