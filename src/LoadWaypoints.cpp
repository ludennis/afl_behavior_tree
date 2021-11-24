#include <LoadWaypoints.h>

namespace AFL
{

namespace
{

geometry_msgs::Pose ParseLineToPose(
    std::string &line, const char delimiter)
{
  std::vector<double> parsedResult;
  std::istringstream ss(line);
  std::string token;
  double d;

  while (std::getline(ss, token, delimiter))
  {
    std::stringstream(token) >> d;
    parsedResult.push_back(d);
  }

  geometry_msgs::Pose pose;
  pose.position.x = parsedResult[0];
  pose.position.y = parsedResult[1];
  pose.position.z = parsedResult[2];
  pose.orientation.x = parsedResult[3];
  pose.orientation.y = parsedResult[4];
  pose.orientation.z = parsedResult[5];
  pose.orientation.w = parsedResult[6];

  return pose;
}

} // namespace

LoadWaypoints::LoadWaypoints(const std::string &name,
    const BT::NodeConfiguration &config)
: SyncActionNode(name, config)
, mNodeHandle("~")
{}

BT::PortsList LoadWaypoints::providedPorts()
{
  return {
      BT::OutputPort<geometry_msgs::PoseArray>("Waypoints"),
  };
}

BT::NodeStatus LoadWaypoints::tick()
{
  mNodeHandle.getParam("waypoints_file_path", mWaypointsFilePath);

  if (mWaypointsFilePath.empty())
  {
    throw BT::RuntimeError(
        "Missing ros param waypoints_file_path", mWaypointsFilePath);
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO_STREAM("[afl_behavior_tree] Loading waypoint file: "
      << mWaypointsFilePath);

  std::string line;
  std::fstream file(mWaypointsFilePath, std::fstream::in);

  if (file.is_open())
  {
    ROS_INFO("Opened file!");
    while (getline(file, line))
    {
      mWaypoints.poses.push_back(ParseLineToPose(line, ','));
    }

    ROS_INFO("Finished parsing");
  }
  else
  {
    throw BT::RuntimeError(
        "File doesn't exists: ", mWaypointsFilePath);
    return BT::NodeStatus::FAILURE;
  }

  file.close();

  setOutput("Waypoints", mWaypoints);

  return BT::NodeStatus::SUCCESS;
}

} // namespace AFL
