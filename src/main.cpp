#include "../include/dv-processing-driver/capture_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dv_capture_node::CaptureNode>());
  rclcpp::shutdown();
  return 0;
}