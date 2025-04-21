#ifndef CUSTOM_NODE_HPP
#define CUSTOM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
/* Include custom_msg */
#include <custom_pkg/msg/custom.hpp>

class CustomNode : public rclcpp::Node {
public:
  CustomNode();

private:
  /**
   * @brief Timer for periodically publishing hello_world
   */
  rclcpp::TimerBase::SharedPtr _timer_info;

  /**
   * @brief Publisher instance for the /hello_world topic
   */
  rclcpp::Publisher<custom_pkg::msg::Custom>::SharedPtr _hello_world_publisher;

  /**
   * @brief Function that publishes hello_world to /hello_world topic
   */
  void publish_hello_world();
};

#endif /* CUSTOM_NODE_HPP */