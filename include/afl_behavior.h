#ifndef _AFL_BEHAVIOR_H_
#define _AFL_BEHAVIOR_H_

#include "shared_class.h"

namespace AFL
{
  class Move2Goal : public Navigation
  {
    public:
      Move2Goal(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      void getParam();
      void posefrontCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
      void posebackCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
      bool poseChecker(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

      ros::Subscriber detected_front;
      ros::Subscriber detected_back;
      std::string pose_front_topic;
      std::string pose_back_topic;

      bool front_isPoseValid;
      bool back_isPoseValid;
      float pallet_height;
      geometry_msgs::Pose buffer, front_pose;
      GoalPose Pose;
  };

  class Move2Height : public Forkctrl
  {
    public:
      Move2Height(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };

  class ForkHigher : public Forkctrl
  {
    public:
      ForkHigher(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };

  class MoveForward : public Navigation
  {
    public:
      MoveForward(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      void PalletFrontEndChecker();
      void opticalSensorCallback(const std_msgs::Bool arrival);
      Optional<GoalPose> Pose;
      InsertionController insertion_controller;
      bool isPalletFrontEndArrival;
  };

  class MoveBackward : public Navigation
  {
    public:
      MoveBackward(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;

    private:
      Optional<GoalPose> Pose;
  };

  class ForkDown : public Forkctrl
  {
    public:
      ForkDown(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };

  class ForkLower : public Forkctrl
  {
    public:
      ForkLower(const std::string& name, const NodeConfiguration& config);
      static BT::PortsList providedPorts();
      BT::NodeStatus tick() override;
  };

  class NormalDetection : public ExtendedNode
  {
    public:
      NormalDetection(const std::string& name, const NodeConfiguration& config);
      BT::NodeStatus tick() override;

    protected:
      ros::NodeHandle n;
  };
}

#endif // _AFL_BEHAVIOR_
