#include "rclcpp/rclcpp.hpp"
#include "shikaku/pybullet_publisher.hpp"

using namespace shikaku;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pybullet_publisher");
  auto pybullet_publisher = std::make_shared<shikaku::PybulletPublisher>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}