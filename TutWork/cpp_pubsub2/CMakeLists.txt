cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(laserscan src/laserscanprocessor.cpp)
ament_target_dependencies(laserscan rclcpp std_msgs sensor_msgs)


install(TARGETS
laserscan
DESTINATION lib/${PROJECT_NAME})

ament_package()
