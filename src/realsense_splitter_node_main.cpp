#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "realsense_utils/realsense_splitter_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<realsense_utils::RealsenseSplitterNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
