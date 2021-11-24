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
{}

BT::PortsList LoadWaypoints::providedPorts()
{
  return {
      BT::InputPort<std::string>("FilePath"),
      BT::OutputPort<geometry_msgs::PoseArray>("Waypoints"),
  };
}

BT::NodeStatus LoadWaypoints::tick()
{
  auto filePath = getInput<std::string>("FilePath");

  if (!filePath)
  {
    throw BT::RuntimeError(
        "Missing required input FilePath: ", filePath.error());
    return BT::NodeStatus::FAILURE;
  }

  std::string line;
  std::fstream file(filePath.value(), std::fstream::in);

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
  }

  file.close();

  setOutput("Waypoints", mWaypoints);

  return BT::NodeStatus::SUCCESS;
}

} // namespace AFL
