cmake_minimum_required(VERSION 3.8)
project(mesh_ping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ping.msg"
)
# Foxy-style linking: rosidl_get_typesupport_target / rclcpp::rclcpp only exist from Humble on.
add_executable(ping_node src/ping_node.cpp)
ament_target_dependencies(ping_node rclcpp)
rosidl_target_interfaces(ping_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pong_node src/pong_node.cpp)
ament_target_dependencies(pong_node rclcpp)
rosidl_target_interfaces(pong_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ping_node pong_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
