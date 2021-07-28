#ifndef _WAIT_FOR_PALLET_DETECTION_H_
#define _WAIT_FOR_PALLET_DETECTION_H_

#include <shared_class.h>

namespace AFL
{
  class WaitForPalletDetection : public ExtendedNode
  {
   public:
    WaitForPalletDetection(const std::string &name,
        const NodeConfiguration &config);
    BT::NodeStatus tick() override;
    static PortsList providedPorts();

   private:
    ros::NodeHandle mNodeHandle;
    Optional<std::string> mPalletDetectionTopic;
  };
} // namespace AFL

#endif // _WAIT_FOR_PALLET_DETECTION_H_
