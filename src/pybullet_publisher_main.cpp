#include "rclcpp/rclcpp.hpp"
#include "shikaku/pybullet_publisher.hpp"

using namespace shikaku;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PybulletPublisher>());
  rclcpp::shutdown();
  return 0;
}