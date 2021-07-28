#include <waitForPalletDetection.h>

namespace AFL
{
  WaitForPalletDetection::WaitForPalletDetection(
      const std::string &name, const NodeConfiguration &config)
  : ExtendedNode(name, config)
  {
    mPalletDetectionTopic = getInput<std::string>("PalletDetectionTopic");
  }

  BT::NodeStatus WaitForPalletDetection::tick()
  {
    auto palletPose =
        ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
            mPalletDetectionTopic.value_or("empty"), ros::Duration(10));

    if (palletPose)
    {
      setOutput("PalletPose", *palletPose);
      ROS_INFO_STREAM_NAMED("[AFL|afl_behavior_tree|WaitForPalletDetection]",
          "Received pallet detection with its pose = "
          << palletPose->pose.pose);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR_NAMED("[AFL|afl_behavior_tree|WaitForPalletDetection]",
          "Pallet detection pose not received");
      return BT::NodeStatus::FAILURE;
    }
  }

  PortsList WaitForPalletDetection::providedPorts()
  {
    return {
      InputPort<std::string>("PalletDetectionTopic"),
      OutputPort<geometry_msgs::PoseWithCovarianceStamped>("PalletPose")
    };
  }
} // namespace AFL
