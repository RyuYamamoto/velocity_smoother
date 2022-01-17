#include <velocity_smoother/velocity_smoother.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocitySmoother>());
  rclcpp::shutdown();
  return 0;
}
