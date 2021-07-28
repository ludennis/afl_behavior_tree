#ifndef _NORMAL_DETECTION_H_
#define _NORMAL_DETECTION_H_

#include <shared_class.h>

namespace AFL
{
  class NormalDetection : public ExtendedNode
  {
    public:
      NormalDetection(const std::string& name, const NodeConfiguration& config);
      BT::NodeStatus tick() override;

    protected:
      ros::NodeHandle n;
  };
} // namespace AFL
#endif // _NORMAL_DETECTION_H_
