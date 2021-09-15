#include <ForkTickDescend.h>

namespace AFL
{

ForkTickDescend::ForkTickDescend(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList ForkTickDescend::providedPorts()
{
  return {
      BT::InputPort<double>("TargetHeight"),
      BT::InputPort<double>("TargetHeightOffset")
  };
}

BT::NodeStatus ForkTickDescend::tick()
{
  auto targetHeight = getInput<double>("TargetHeight");
  auto targetHeightOffset = getInput<double>("TargetHeightOffset");

  if (!targetHeight)
  {
    throw BT::RuntimeError(
        "Missing required input TargetHeight: ", targetHeight.error());
    return BT::NodeStatus::FAILURE;
  }

  auto currentHeight = ros::topic::waitForMessage<std_msgs::Float64>(
      "/afl/fork_height", ros::Duration(10));

  if (!currentHeight)
  {
    throw BT::RuntimeError(
        "Missing required input TargetHeight: ", targetHeight.error());
    return BT::NodeStatus::FAILURE;
  }

  ros::Rate twoSecond(0.5);
  while (currentHeight->data >
      targetHeight.value() + targetHeightOffset.value_or(0.0))
  {
    TickDescend();
    twoSecond.sleep();
    currentHeight = ros::topic::waitForMessage<std_msgs::Float64>(
        "/afl/fork_height", ros::Duration(10));
    if (!currentHeight)
    {
      throw BT::RuntimeError(
          "Missing required input TargetHeight: ", targetHeight.error());
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

void ForkTickDescend::TickDescend()
{
  ros::ServiceClient setForkDownClient =
      mNodeHandle.serviceClient<aflctrl_msgs::setForkDown>("/afl/setForkDown");
  ros::ServiceClient setForkStopClient =
      mNodeHandle.serviceClient<aflctrl_msgs::setForkStop>("/afl/setForkStop");
  aflctrl_msgs::setForkDown setForkDownMessage;
  aflctrl_msgs::setForkStop setForkStopMessage;

  setForkDownClient.call(setForkDownMessage);
  setForkStopClient.call(setForkStopMessage);
}

} // namespace AFL
