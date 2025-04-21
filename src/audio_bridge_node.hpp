/**
 * @file audio_bridge_node.hpp
 * @brief Bridge between audio_tools and ros2_whisper
 *
 * This node subscribes to audio_tools's AudioDataStamped messages and
 * republishes them in the format expected by ros2_whisper's inference node.
 */
#ifndef WHISPER_BRIDGE__AUDIO_BRIDGE_NODE_HPP
#define WHISPER_BRIDGE__AUDIO_BRIDGE_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include "whisper_bridge/msg/audio_data_stamped.hpp"
// #include "audio_tools/msg/audio_data_stamped.hpp"

class AudioBridgeNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the AudioBridgeNode
   * Initializes publishers and subscribers and sets up parameters
   */
  AudioBridgeNode();

private:
  /**
   * @brief Callback for audio data messages from audio_tools
   * @param msg The audio message received from audio_tools
   */
  void audio_callback(const whisper_bridge::msg::AudioDataStamped::SharedPtr msg);
  
  /**
   * @brief Periodically check for subscriber connections
   */
  void check_connections();

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr whisper_audio_pub_;
  rclcpp::Subscription<whisper_bridge::msg::AudioDataStamped>::SharedPtr audio_tools_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // WHISPER_BRIDGE__AUDIO_BRIDGE_NODE_HPP