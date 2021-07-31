#ifndef _VELOCITY_SMOOTHER_
#define _VELOCITY_SMOOTHER_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class VelocitySmoother
{
public:
  VelocitySmoother();
  ~VelocitySmoother() = default;

private:
  void twistCallback(const geometry_msgs::Twist msg);
  void timerCallback(const ros::TimerEvent & e);
  geometry_msgs::Twist filteredTwist(const geometry_msgs::Twist twist);

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::Subscriber raw_cmd_vel_subscriber_;
  ros::Publisher filtered_cmd_vel_publisher_;

  geometry_msgs::Twist target_cmd_vel_;
  geometry_msgs::Twist last_cmd_vel_;

  ros::Time last_time_stamp_;

  ros::Timer timer_;

  double velocity_gain_;
  double angular_gain_;

  double maximum_limit_vel_;
  double minimum_limit_vel_;
};

#endif
