cmake_minimum_required(VERSION 3.1...3.14)
project(custom_pkg C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
 msg/Custom.msg
 DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(custom_node src/custom_node.cpp)

ament_target_dependencies(custom_node
   rclcpp
   std_msgs
)
target_link_libraries(custom_node "${cpp_typesupport_target}")

if(UNIX AND NOT APPLE)  # Linux only
 target_compile_options(custom_node PRIVATE -Wall)
endif()

install(TARGETS
custom_node
DESTINATION lib/${PROJECT_NAME}) 

ament_package()
