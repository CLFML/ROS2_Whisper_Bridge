cmake_minimum_required(VERSION 3.14)
project(whisper_bridge C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AudioData.msg
  msg/AudioDataStamped.msg
  msg/AudioInfo.msg
  msg/VoiceActivity.msg  
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(audio_bridge_node src/audio_bridge_node.cpp)
target_link_libraries(audio_bridge_node "${cpp_typesupport_target}")

if(UNIX AND NOT APPLE)  # Linux only
 target_compile_options(audio_bridge_node PRIVATE -Wall)
endif()

ament_target_dependencies(audio_bridge_node
   rclcpp
   std_msgs
)

# Install targets
install(TARGETS
audio_bridge_node
DESTINATION lib/${PROJECT_NAME}) 
# DESTINATION share/${PROJECT_NAME}/launch ??

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
