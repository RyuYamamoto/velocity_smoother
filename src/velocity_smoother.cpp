#include <velocity_smoother/velocity_smoother.h>

VelocitySmoother::VelocitySmoother()
{
  pnh_.param<double>("velocity_gain", velocity_gain_, 1.0);
  pnh_.param<double>("angular_gain", angular_gain_, 1.0);
  pnh_.param<double>("maximum_limit_vel", maximum_limit_vel_, 0.5);
  pnh_.param<double>("minimum_limit_vel", minimum_limit_vel_, -0.5);

  raw_cmd_vel_subscriber_ =
    pnh_.subscribe("raw_cmd_vel", 1, &VelocitySmoother::twistCallback, this);
  filtered_cmd_vel_publisher_ = pnh_.advertise<geometry_msgs::Twist>("filtered_cmd_vel", 1);

  last_time_stamp_ = ros::Time::now();

  timer_ = nh_.createTimer(ros::Duration(0.01), &VelocitySmoother::timerCallback, this);
}

void VelocitySmoother::twistCallback(const geometry_msgs::Twist msg)
{
  // check velocity limit
  target_cmd_vel_.linear.x = msg.linear.x > 0.0 ? std::min(msg.linear.x, maximum_limit_vel_)
                                                : std::max(msg.linear.x, minimum_limit_vel_);
  target_cmd_vel_.angular.z = msg.angular.x > 0.0 ? std::min(msg.angular.z, maximum_limit_vel_)
                                                  : std::max(msg.angular.z, minimum_limit_vel_);
}

void VelocitySmoother::timerCallback(const ros::TimerEvent & e)
{
  geometry_msgs::Twist cmd_vel;

  ros::Time current_time_stamp = ros::Time::now();
  const double dt = (current_time_stamp - last_time_stamp_).toSec();

  const double delta_linear_vel =
    velocity_gain_ * (target_cmd_vel_.linear.x - last_cmd_vel_.linear.x);
  const double delta_angular_vel =
    angular_gain_ * (target_cmd_vel_.angular.z - last_cmd_vel_.angular.z);

  cmd_vel.linear.x = delta_linear_vel * dt + last_cmd_vel_.linear.x;
  cmd_vel.angular.z = delta_angular_vel * dt + last_cmd_vel_.angular.z;

  filtered_cmd_vel_publisher_.publish(cmd_vel);

  last_cmd_vel_ = cmd_vel;
  last_time_stamp_ = current_time_stamp;
}
