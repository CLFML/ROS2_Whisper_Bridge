#include "custom_node.hpp"

CustomNode::CustomNode() : Node("custom_node") {
  RCLCPP_INFO(this->get_logger(), "Custom node has been started.");
  _hello_world_publisher =
      this->create_publisher<custom_pkg::msg::Custom>("hello_world", 10);
  _timer_info = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(1),
                                     [this] { publish_hello_world(); });
}

void CustomNode::publish_hello_world() {
  custom_pkg::msg::Custom msg;
  msg.custom_msg = "Hello World!";
  _hello_world_publisher->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomNode>());
  rclcpp::shutdown();
  return 0;
}
