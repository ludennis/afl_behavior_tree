#include <LoadWaypoints.h>

namespace AFL
{

namespace
{

std::vector<std::string> ParseLine(
    std::string &line, const std::string &delimiter)
{
  std::vector<std::string> parsedResult;

  size_t pos = 0;
  std::string token;
  while ((pos = line.find(delimiter)) != std::string::npos)
  {
    token = line.substr(0, pos);
    parsedResult.push_back(token);
    line.erase(0, pos + delimiter.length());
  }
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
    while (getline(file, line))
    {
      auto parsed = ParseLine(line, ",");

      geometry_msgs::Pose pose;
      pose.position.x = stod(parsed[0]);
      pose.position.y = stod(parsed[1]);
      pose.position.z = stod(parsed[2]);
      pose.orientation.x = stod(parsed[3]);
      pose.orientation.y = stod(parsed[4]);
      pose.orientation.z = stod(parsed[5]);
      pose.orientation.w = stod(parsed[6]);

      mWaypoints.poses.push_back(pose);
    }

    file.close();

    setOutput("Waypoints", mWaypoints);

    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    throw BT::RuntimeError(
        "File doesn't exists: ", mWaypointsFilePath);
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace AFL
