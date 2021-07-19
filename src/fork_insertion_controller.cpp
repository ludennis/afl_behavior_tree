#include "fork_insertion_controller.h"

namespace AFL
{
  InsertionController::InsertionController()
  {
    ros::param::get("~fork_insertion_controller/ackermann_speed", forward_speed);
    speed_pub = n.advertise<ackermann_msgs::AckermannDrive>("/afl/cmd_ackermann", 100);
  }

  void InsertionController::setForward(bool isArrival)
  {
    ROS_INFO_STREAM_NAMED("AFL", "[fork_insertion_controller] get ackermann_speed: " <<
        forward_speed);

    if(!isArrival)
      publishAckermannSpeed(forward_speed);
    else
      publishAckermannSpeed(0);

  }

  void InsertionController::publishAckermannSpeed(double speed)
  {
    ROS_INFO_STREAM_NAMED("AFL", "[fork_insertion_controller] publish ackermann_speed: " <<
        speed);
    ackermann_msgs.speed = forward_speed;
    ackermann_msgs.steering_angle = 0;
    speed_pub.publish(ackermann_msgs);
  }
}

