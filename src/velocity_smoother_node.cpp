#include <velocity_smoother/velocity_smoother.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "velocity_smoother_node");

  VelocitySmoother velocity_smoother;

  ros::spin();

  return 0;
}
