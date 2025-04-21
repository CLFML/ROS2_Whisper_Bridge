/**
 * @file audio_bridge_node.cpp
 * @brief Bridge between audio_tools and ros2_whisper
 *
 * This node subscribes to audio_tools's AudioDataStamped messages and
 * republishes them in the format expected by ros2_whisper's inference node.
 */
#include "audio_bridge_node.hpp"

AudioBridgeNode::AudioBridgeNode() : Node("audio_bridge_node") {
  // Declare parameters
  this->declare_parameter<std::string>("input_topic", "/audio_stamped");
  this->declare_parameter<std::string>("output_topic", "/audio_listener/audio");
  
  // Get parameters
  std::string input_topic, output_topic;
  this->get_parameter("input_topic", input_topic);
  this->get_parameter("output_topic", output_topic);
  
  RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic.c_str());
  
  // Create publisher for whisper-compatible format with reliable QoS for network transmission
  whisper_audio_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      output_topic, 
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  
  // Create subscription to audio_tools format
  audio_tools_sub_ = this->create_subscription<whisper_bridge::msg::AudioDataStamped>(
      input_topic, 10, 
      std::bind(&AudioBridgeNode::audio_callback, this, std::placeholders::_1));
  
  // Create timer for connection checking
  timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&AudioBridgeNode::check_connections, this));
  
  RCLCPP_INFO(this->get_logger(), "Audio bridge node initialized");
}

void AudioBridgeNode::audio_callback(const whisper_bridge::msg::AudioDataStamped::SharedPtr msg) {
  // Check that the format is compatible (whisper expects S16LE)
  if (msg->info.sample_format != "S16LE") {
    RCLCPP_WARN(this->get_logger(), 
               "Received audio in format %s, but whisper expects S16LE. Some data may be lost.",
               msg->info.sample_format.c_str());
  }
  
  // Check that channels is compatible (whisper expects mono)
  if (msg->info.channels != 1) {
    RCLCPP_WARN_ONCE(this->get_logger(), 
                   "Received audio with %d channels, but whisper expects mono. Some data may be lost.",
                   msg->info.channels);
  }
  
  // The raw audio data
  const auto& audio_data = msg->audio.data;
  
  // Skip empty messages
  if (audio_data.empty()) {
    return;
  }
  
  // Create Int16MultiArray message for whisper
  auto whisper_msg = std::make_unique<std_msgs::msg::Int16MultiArray>();
  
  // Set the layout
  whisper_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  whisper_msg->layout.dim[0].label = "audio";
  whisper_msg->layout.dim[0].size = audio_data.size() / 2; // Each S16LE sample is 2 bytes
  whisper_msg->layout.dim[0].stride = 1;
  whisper_msg->layout.data_offset = 0;
  
  // Convert audio data to int16
  whisper_msg->data.resize(audio_data.size() / 2);
  
  // Copy data (assumes S16LE format)
  for (size_t i = 0; i < whisper_msg->data.size(); i++) {
    // Convert two bytes to a 16-bit sample (little endian)
    whisper_msg->data[i] = static_cast<int16_t>(
      (static_cast<uint16_t>(audio_data[i*2+1]) << 8) | 
      static_cast<uint16_t>(audio_data[i*2]));
  }
  
  // Publish the converted message
  whisper_audio_pub_->publish(std::move(whisper_msg));
}

void AudioBridgeNode::check_connections() {
  // Check if there are any subscribers to our published topic
  if (whisper_audio_pub_->get_subscription_count() == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
      "No subscribers to whisper audio topic. Is the server running?");
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AudioBridgeNode>());
  rclcpp::shutdown();
  return 0;
}