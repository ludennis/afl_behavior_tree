#include <DetectionClient.h>

namespace AFL
{

DetectionClient::DetectionClient(const std::string &name,
    const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList DetectionClient::providedPorts()
{
  return {
      BT::InputPort<std::string>("Operation")
  };
}

BT::NodeStatus DetectionClient::tick()
{
  auto operation = getInput<std::string>("Operation");

  if (!operation)
  {
    throw BT::RuntimeError(
        "Missing required input Operatrion: ", operation.error());
    return BT::NodeStatus::FAILURE;
  }

  if (operation.value() == "enable" || operation.value() == "Enable" ||
      operation.value() == "ENABLE")
  {
    auto client = mNodeHandle.serviceClient<afl_detection::ActivateDetection>(
        "activate_detection");
    afl_detection::ActivateDetection msg;
    if (!client.call(msg))
    {
      ROS_ERROR("[afl_behavior_tree] Failed to enable detection");
      return BT::NodeStatus::FAILURE;
    }
    ROS_INFO("[afl_behavior_tree] Enabled Detection");
  }
  else
  {
    auto client = mNodeHandle.serviceClient<afl_detection::DeactivateDetection>(
        "deactivate_detection");
    afl_detection::DeactivateDetection msg;
    if (!client.call(msg))
    {
      ROS_ERROR("[afl_behavior_tree] Failed to disable detection");
      return BT::NodeStatus::FAILURE;
    }
    ROS_INFO("[afl_behavior_tree] Disabled Detection");
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace AFL
