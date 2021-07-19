#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace AFL
{
  class InsertionController
  {
    public:
      InsertionController();
      void setForward(bool isArrival);

    private:
      ackermann_msgs::AckermannDrive ackermann_msgs;
      ros::NodeHandle n;
      ros::Publisher speed_pub;
      double forward_speed;
      void publishAckermannSpeed(double speed);
  };
}
