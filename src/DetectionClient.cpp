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
        "activateDetection");
    afl_detection::ActivateDetection msg;
    if (!client.call(msg))
      return BT::NodeStatus::FAILURE;
  }
  else
  {
    auto client = mNodeHandle.serviceClient<afl_detection::DeactivateDetection>(
        "deactivateDetection");
    afl_detection::DeactivateDetection msg;
    if (!client.call(msg))
      return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

} // namespace AFL
