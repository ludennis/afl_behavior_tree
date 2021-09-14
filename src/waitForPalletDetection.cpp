#include <waitForPalletDetection.h>

namespace AFL
{

WaitForPalletDetection::WaitForPalletDetection(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{
  mPalletTfName = getInput<std::string>("PalletTfName");
  mMapTfName= getInput<std::string>("MapTfName");
}

BT::NodeStatus WaitForPalletDetection::tick()
{
  tf::StampedTransform stampedTransform;
  tf::TransformListener tfListener;
  tfListener.waitForTransform(mMapTfName.value(), mPalletTfName.value(), ros::Time(),
      ros::Duration(120.0));

  try
  {
    tfListener.lookupTransform(mMapTfName.value(), mPalletTfName.value(), ros::Time(),
        stampedTransform);

    double yaw, pitch, roll;
    stampedTransform.getBasis().getRPY(roll, pitch, yaw);
    tf::Quaternion q = stampedTransform.getRotation();
    tf::Vector3 v = stampedTransform.getOrigin();
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ()
        << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
        << q.getZ() << ", " << q.getW() << "]" << std::endl
        << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw
        << "]" << std::endl
        << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI
        << ", " << yaw*180.0/M_PI << "]" << std::endl;

    // TODO: remove this after pallet detectin is fixed
    v.setY(v.getY() - 0.1);
    stampedTransform.setOrigin(v);

    setOutput("PalletPose", stampedTransform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  setOutput("PalletPose", stampedTransform);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList WaitForPalletDetection::providedPorts()
{
  return {
    BT::InputPort<std::string>("PalletTfName"),
    BT::InputPort<std::string>("MapTfName"),
    BT::OutputPort<tf::StampedTransform>("PalletPose")
  };
}

} // namespace AFL
