#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // This executable currently does not run a node directly.
  // It exists to satisfy the build and act as a placeholder
  // for future system-wide coordination or launch wiring.
  RCLCPP_INFO(rclcpp::get_logger("sailbot_main"),
              "sailbot_main started (no internal nodes).");

  // Spin a dummy node so the process remains active if desired.
  auto node = std::make_shared<rclcpp::Node>("sailbot_main");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
