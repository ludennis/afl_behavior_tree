#ifndef _MOVE_TO_GOAL_H_
#define _MOVE_TO_GOAL_H_

#include <shared_class.h>

namespace AFL
{
  class MoveToGoal : public Navigation
  {
    public:
      MoveToGoal(const std::string& name, const NodeConfiguration& config);
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
} // namespace AFL

#endif // _MOVE_TO_GOAL_H_
