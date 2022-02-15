#include <WaitForAprilTagDetection.h>

namespace AFL
{

WaitForAprilTagDetection::WaitForAprilTagDetection(
    const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{
  mAprilTagTfName = getInput<std::string>("AprilTagTfName");
  mMapTfName = getInput<std::string>("MapTfName");
}

BT::NodeStatus WaitForAprilTagDetection::tick()
{
  tf::StampedTransform stampedTransform;
  tf::TransformListener tfListener;
  tfListener.waitForTransform(mMapTfName.value(), mAprilTagTfName.value(),
      ros::Time(), ros::Duration(120.0));

  try
  {
    tfListener.lookupTransform(mMapTfName.value(), mAprilTagTfName.value(),
        ros::Time(), stampedTransform);

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

    setOutput("AprilTagPose", stampedTransform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  setOutput("PalletPose", stampedTransform);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList WaitForAprilTagDetection::providedPorts()
{
  return {
    BT::InputPort<std::string>("AprilTagTfName"),
    BT::InputPort<std::string>("MapTfName"),
    BT::OutputPort<tf::StampedTransform>("AprilTagPose")
  };
}

} // namespace AFL
